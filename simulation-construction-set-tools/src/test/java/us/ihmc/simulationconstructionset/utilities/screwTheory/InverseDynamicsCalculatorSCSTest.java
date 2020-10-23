package us.ihmc.simulationconstructionset.utilities.screwTheory;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsMechanismExplorer;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This currently needs to be here because it uses SCS classes to test the inverse dynamics calculator, and SCS isn't on the IHMCUtilities build path
 * @author Twan Koolen
 *
 */
public class InverseDynamicsCalculatorSCSTest
{
   private static final boolean EXPLORE_AND_PRINT = false;
   
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);
   
   
   private final Random random = new Random(100L);

   @BeforeEach
   public void setUp()
   {
   }

   @Test
   public void testOneFreeRigidBody()
   {
      Robot robot = new Robot("robot");
      double gravity = -9.81;
      robot.setGravity(gravity);

      ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", inertialFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();

      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3D(0.1, 0.2, 0.3), robot);
      robot.addRootJoint(rootJoint);

      SixDoFJoint rootInverseDynamicsJoint = new SixDoFJoint("root", elevator);

      Link link = createRandomLink("link", false);
      rootJoint.setLink(link);
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
//      System.out.println("comOffset = " + comOffset);

      RigidBodyBasics body = copyLinkAsRigidBody(link, rootInverseDynamicsJoint, "body");

      setRandomPosition(rootJoint, rootInverseDynamicsJoint);

      setRandomVelocity(rootJoint, rootInverseDynamicsJoint);
      elevator.updateFramesRecursively();

      ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      
      TranslationReferenceFrame forceApplicationFrame = new TranslationReferenceFrame("forceApplicationFrame", bodyFixedFrame);
      Vector3D translationFromCoM = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      forceApplicationFrame.updateTranslation(translationFromCoM);
      
      Vector3D externalForcePointOffset = new Vector3D(comOffset);
      externalForcePointOffset.add(translationFromCoM);
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("rootExternalForcePoint", externalForcePointOffset, robot.getRobotsYoRegistry());
      rootJoint.addExternalForcePoint(externalForcePoint);
      externalForcePoint.setForce(random.nextDouble(), random.nextDouble(), random.nextDouble());

      Vector3D externalForce = new Vector3D();
      externalForcePoint.getForce(externalForce); // in world frame
      RigidBodyTransform worldToBody = elevatorFrame.getTransformToDesiredFrame(forceApplicationFrame);
      worldToBody.transform(externalForce);

      Wrench inputWrench = new Wrench(forceApplicationFrame, forceApplicationFrame, new Vector3D(), externalForce);
      doRobotDynamics(robot);

      copyAccelerationFromForwardToInverse(rootJoint, rootInverseDynamicsJoint);

      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(elevator);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      inverseDynamicsCalculator.compute();
      inverseDynamicsCalculator.writeComputedJointWrenches(SubtreeStreams.fromChildren(elevator).collect(Collectors.toList()));

      Wrench outputWrench = new Wrench(rootInverseDynamicsJoint.getJointWrench());
      
      outputWrench.setBodyFrame(forceApplicationFrame);
      outputWrench.changeFrame(forceApplicationFrame);

      compareWrenches(inputWrench, outputWrench);
   }

   @Test
   public void testChainNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      Vector3D[] jointAxes = {X, Y, Z, X};
      
      double gravity = 0.0;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, true, true, random);
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, true, true);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

   @Test
   public void testTreeWithNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      double gravity = 0.0;

      int numberOfJoints = 3;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true, random);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, true, true);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

   @Test
   public void testTreeWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      double gravity = -9.8;

      int numberOfJoints = 100;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true, random);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, true, true);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

   @Test
   public void testDoingInverseDynamicsTermPerTerm()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      LinkedHashMap<RevoluteJoint, Double> tau_gravity = new LinkedHashMap<RevoluteJoint, Double>();
      LinkedHashMap<RevoluteJoint, Double> tau_cc = new LinkedHashMap<RevoluteJoint, Double>();
      LinkedHashMap<RevoluteJoint, Double> tau_qdd = new LinkedHashMap<RevoluteJoint, Double>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      double gravity = -9.8;

      int numberOfJoints = 100;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true, random);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }

      // gravity term
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, false, false);
      for (RevoluteJoint joint : jointMap.keySet())
         tau_gravity.put(joint, joint.getTau());
      // coriolis/centrifugal term
      createInverseDynamicsCalculatorAndCompute(elevator, 0.0, worldFrame, true, false);
      for (RevoluteJoint joint : jointMap.keySet())
         tau_cc.put(joint, joint.getTau());
      // mass matrix times desired accelerations term
      createInverseDynamicsCalculatorAndCompute(elevator, 0.0, worldFrame, false, true);
      for (RevoluteJoint joint : jointMap.keySet())
         tau_qdd.put(joint, joint.getTau());

      for (RevoluteJoint joint : jointMap.keySet())
         joint.setTau(tau_qdd.get(joint) + tau_cc.get(joint) + tau_gravity.get(joint));
      
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

   @Test
   public void testDoingNothing()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      double gravity = -9.8;

      int numberOfJoints = 100;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true, random);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }

      createInverseDynamicsCalculatorAndCompute(elevator, 0.0, worldFrame, false, false);
      
      double epsilon = 1e-12;
      for (RevoluteJoint joint : jointMap.keySet())
      {
         double tau = joint.getTau();
         assertEquals(0.0, tau, epsilon);
      }
   }

   @Test
   public void testGravityCompensationForChain()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      Vector3D[] jointAxes = {X, Y, Z, X};
      double gravity = -9.8;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, false, false, random);
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, false, false);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertZeroAccelerations(jointMap);
   }

   @Test
   public void testChainWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      Vector3D[] jointAxes = {X, Y, Z, X};
      
      double gravity = -9.8;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, true, true, random);
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, true, true);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

   
   private void exploreAndPrintRobot(Robot robot)
   {
      RobotExplorer robotExplorer = new RobotExplorer(robot);
      StringBuffer buffer = new StringBuffer();
      robotExplorer.getRobotInformationAsStringBuffer(buffer);
      System.out.println("-----------------------------");
      System.out.println(buffer);
      System.out.println("-----------------------------");
   }
   
   private void exploreAndPrintInverseDynamicsMechanism(RigidBodyBasics elevator)
   {
      InverseDynamicsMechanismExplorer idMechanismExplorer = new InverseDynamicsMechanismExplorer(elevator);
      System.out.println("-----------------------------");
      System.out.println(idMechanismExplorer);
      System.out.println("-----------------------------");
   }
   
   private InverseDynamicsCalculator createInverseDynamicsCalculatorAndCompute(RigidBodyBasics elevator, double gravity, ReferenceFrame worldFrame, boolean doVelocityTerms, boolean doAcceleration)
   {
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(elevator, doVelocityTerms, doAcceleration);
      inverseDynamicsCalculator.setGravitionalAcceleration(gravity);
      inverseDynamicsCalculator.compute();
      inverseDynamicsCalculator.writeComputedJointWrenches(SubtreeStreams.fromChildren(elevator).collect(Collectors.toList()));
      return inverseDynamicsCalculator;
   }
   
   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getReferenceFrame().checkReferenceFrameMatch(outputWrench.getReferenceFrame());

      double epsilon = 1e-12; //3;
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(inputWrench.getAngularPart()), new Vector3D(outputWrench.getAngularPart()), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(inputWrench.getLinearPart()), new Vector3D(outputWrench.getLinearPart()), epsilon);
   }
   
   private static void copyTorques(HashMap<RevoluteJoint, PinJoint> jointMap)
   {
      for (RevoluteJoint idJoint : jointMap.keySet())
      {
         OneDegreeOfFreedomJoint joint = jointMap.get(idJoint);
         double tau = idJoint.getTau();
//         System.out.println("tau = " + tau);
         
         joint.setTau(tau);
      }
   }

   private void assertAccelerationsEqual(HashMap<RevoluteJoint, PinJoint> jointMap)
   {
      double epsilon = 1e-12;
      for (RevoluteJoint idJoint : jointMap.keySet())
      {
         OneDegreeOfFreedomJoint revoluteJoint = jointMap.get(idJoint);

         YoDouble qddVariable = revoluteJoint.getQDDYoVariable();
         double qdd = qddVariable.getDoubleValue();
         double qddInverse = idJoint.getQdd();

//         YoDouble tauVariable = revoluteJoint.getTau();
//         System.out.println("qddInverse: " + qddInverse + ", qdd: " + qdd);
//         System.out.println("tau: "  + ", tauVariable: " + tauVariable.getDoubleValue());
         
         assertEquals(qddInverse, qdd, epsilon);
      }
   }
   
   private void assertZeroAccelerations(HashMap<RevoluteJoint, PinJoint> jointMap)
   {
      double epsilon = 1e-12;
      for (OneDegreeOfFreedomJoint joint : jointMap.values())
      {
         double qdd = joint.getQDDYoVariable().getDoubleValue();
         assertEquals(0.0, qdd, epsilon);
      }
   }

   private void doRobotDynamics(Robot robot) 
   {
      try
      {
         robot.doDynamicsButDoNotIntegrate();
      } 
      catch (UnreasonableAccelerationException e)
      {
         throw new RuntimeException(e);
      }
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(robot, false);
//      scs.disableGUIComponents();
//      scs.setRecordDT(scs.getDT());
//      Thread simThread = new Thread(scs, "InverseDynamicsCalculatorTest sim thread");
//      simThread.start();
//      scs.simulate(1);
//      waitForSimulationToFinish(scs);
   }

   private static void createRandomChainRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBodyBasics elevator, Vector3D[] jointAxes, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration, Random random)
   {
      robot.setGravity(gravity);     

      RigidBodyBasics currentIDBody = elevator;
      PinJoint previousJoint = null;
      for (int i = 0; i < jointAxes.length; i++)
      {         
         Vector3D jointOffset = RandomGeometry.nextVector3D(random);
         Vector3D jointAxis = jointAxes[i];
         Matrix3D momentOfInertia = RandomGeometry.nextDiagonalMatrix3D(random);
         double mass = random.nextDouble();
         Vector3D comOffset = RandomGeometry.nextVector3D(random);
         double jointPosition = random.nextDouble();
         double jointVelocity = useRandomVelocity ? random.nextDouble() : 0.0;
         double jointAcceleration = useRandomAcceleration ? random.nextDouble() : 0.0;
         
         RevoluteJoint currentIDJoint = new RevoluteJoint("jointID" + i, currentIDBody, jointOffset, jointAxis);
         currentIDJoint.setQ(jointPosition);
         currentIDJoint.setQd(jointVelocity);
         currentIDJoint.setQdd(jointAcceleration);
         
         currentIDBody = new RigidBody("bodyID" + i, currentIDJoint, momentOfInertia, mass, comOffset);
         
         PinJoint currentJoint = new PinJoint("joint" + i, jointOffset, robot, jointAxis);
         currentJoint.setInitialState(jointPosition, jointVelocity);
         if (previousJoint == null)
            robot.addRootJoint(currentJoint);
         else
            previousJoint.addJoint(currentJoint);

         Link currentBody = new Link("body" + i);
         currentBody.setComOffset(comOffset);
         currentBody.setMass(mass);
         currentBody.setMomentOfInertia(momentOfInertia);
         currentJoint.setLink(currentBody);
         
         jointMap.put(currentIDJoint, currentJoint);
         previousJoint = currentJoint;
      }

      elevator.updateFramesRecursively();
   }
   
   
   public static void createRandomTreeRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBodyBasics elevator, int numberOfJoints, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration, Random random)
   {
      robot.setGravity(gravity);     
    
      ArrayList<PinJoint> potentialParentJoints = new ArrayList<PinJoint>();
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>(); // synchronized with potentialParentJoints

      
      for (int i = 0; i < numberOfJoints; i++)
      {         
         Vector3D jointOffset = RandomGeometry.nextVector3D(random);
         Vector3D jointAxis = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         jointAxis.normalize();
         Matrix3D momentOfInertia = RandomGeometry.nextDiagonalMatrix3D(random);
         double mass = random.nextDouble();
         Vector3D comOffset = RandomGeometry.nextVector3D(random);
         double jointPosition = random.nextDouble();
         double jointVelocity = useRandomVelocity ? random.nextDouble() : 0.0;
         double jointAcceleration = useRandomAcceleration ? random.nextDouble() : 0.0;

         PinJoint currentJoint = new PinJoint("joint" + i, jointOffset, robot, jointAxis);
         currentJoint.setInitialState(jointPosition, jointVelocity);
         RigidBodyBasics inverseDynamicsParentBody;
         if (potentialParentJoints.isEmpty())
         {
            robot.addRootJoint(currentJoint);
            inverseDynamicsParentBody = elevator;
         }
         else
         {
            int parentIndex = random.nextInt(potentialParentJoints.size());
            potentialParentJoints.get(parentIndex).addJoint(currentJoint);
            RevoluteJoint inverseDynamicsParentJoint = potentialInverseDynamicsParentJoints.get(parentIndex);
            inverseDynamicsParentBody = inverseDynamicsParentJoint.getSuccessor();
         }

         RevoluteJoint currentIDJoint = new RevoluteJoint("jointID" + i, inverseDynamicsParentBody, jointOffset, jointAxis);
         currentIDJoint.setQ(jointPosition);
         currentIDJoint.setQd(jointVelocity);
         currentIDJoint.setQdd(jointAcceleration);
         new RigidBody("bodyID" + i, currentIDJoint, momentOfInertia, mass, comOffset);

         Link currentBody = new Link("body" + i);
         currentBody.setComOffset(comOffset);
         currentBody.setMass(mass);
         currentBody.setMomentOfInertia(momentOfInertia);
         currentJoint.setLink(currentBody);
         
         jointMap.put(currentIDJoint, currentJoint);
         
         potentialParentJoints.add(currentJoint);
         potentialInverseDynamicsParentJoints.add(currentIDJoint);
      }

      elevator.updateFramesRecursively();
   }

   private Link createRandomLink(String linkName, boolean useZeroCoMOffset)
   {
      Link link = new Link(linkName);

      link.setMomentOfInertia(random.nextDouble(), random.nextDouble(), random.nextDouble());
      link.setMass(random.nextDouble());

      Vector3D comOffset = useZeroCoMOffset ? new Vector3D() : new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      link.setComOffset(comOffset);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);
      double mass = link.getMass();
      
      linkGraphics.createInertiaEllipsoid(momentOfInertia, comOffset, mass, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private RigidBodyBasics copyLinkAsRigidBody(Link link, JointBasics currentInverseDynamicsJoint, String bodyName)
   {
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);

      return new RigidBody(bodyName, currentInverseDynamicsJoint, momentOfInertia, link.getMass(), comOffset);
   }

   private void setRandomPosition(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Point3D rootPosition = new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();

      floatingJoint.setPosition(rootPosition);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

      sixDoFJoint.setJointPosition(rootPosition);
      sixDoFJoint.getJointPose().getOrientation().setYawPitchRoll(yaw, pitch, roll);
   }
   
   private final FrameVector3D linearVelocityFrameVector = new FrameVector3D();
   private final FrameVector3D angularVelocityFrameVector = new FrameVector3D();
   
   private void setRandomVelocity(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Vector3D linearVelocity = RandomGeometry.nextVector3D(random, 1.0);
      Vector3D angularVelocity = RandomGeometry.nextVector3D(random, 1.0);

      floatingJoint.setVelocity(linearVelocity);
      floatingJoint.setAngularVelocityInBody(angularVelocity);

      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      floatingJoint.getVelocity(linearVelocityFrameVector);
      linearVelocityFrameVector.changeFrame(bodyFrame);

      floatingJoint.getAngularVelocity(angularVelocityFrameVector, bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, angularVelocityFrameVector, linearVelocityFrameVector);
      sixDoFJoint.setJointTwist(bodyTwist);
   }

   
   private void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      // TODO: Get this to work when the FloatingJoint has an offset.

      Twist bodyTwist = new Twist();
      bodyTwist.setIncludingFrame(sixDoFJoint.getJointTwist());

      FrameVector3D originAcceleration = new FrameVector3D(sixDoFJoint.getFrameBeforeJoint());
      FrameVector3D angularAcceleration = new FrameVector3D(sixDoFJoint.getFrameAfterJoint());

      floatingJoint.getLinearAccelerationInWorld(originAcceleration);
      floatingJoint.getAngularAccelerationInBody(angularAcceleration);
      originAcceleration.changeFrame(sixDoFJoint.getFrameBeforeJoint());

      SpatialAcceleration spatialAccelerationVector = new SpatialAcceleration(sixDoFJoint.getFrameAfterJoint(), sixDoFJoint.getFrameBeforeJoint(), sixDoFJoint.getFrameAfterJoint());
      
      spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      sixDoFJoint.setJointAcceleration(spatialAccelerationVector);
   }

   private static class RobotExplorer
   {
      private final Robot robot;

      public RobotExplorer(Robot robot)
      {
         this.robot = robot;
      }

      public void getRobotInformationAsStringBuffer(StringBuffer buffer)
      {
         List<Joint> rootJoints = robot.getRootJoints();

         for (Joint rootJoint : rootJoints)
         {
            buffer.append("Found Root Joint.\n");
            printJointInformation(rootJoint, buffer);
         }
      }

      private void printJointInformation(Joint joint, StringBuffer buffer)
      {
         String jointName = joint.getName();

         buffer.append("Joint name = " + jointName + "\n");

         Vector3D offset = new Vector3D();
         joint.getOffset(offset);

         buffer.append("Joint offset = " + offset + "\n");

         Vector3D jointAxis = new Vector3D();
         joint.getJointAxis(jointAxis);

         buffer.append("Joint axis = " + jointAxis + "\n");

         if (joint instanceof PinJoint)
         {
            printPinJointInformation((OneDegreeOfFreedomJoint) joint, buffer);
         }

         else if (joint instanceof SliderJoint)
         {
            printSliderJointInformation((SliderJoint) joint, buffer);
         }

         else if (joint instanceof FloatingJoint)
         {
            printFloatingJointInformation((FloatingJoint) joint, buffer);
         }

         else if (joint instanceof FloatingPlanarJoint)
         {
            printFloatingPlanarJointInformation((FloatingPlanarJoint) joint, buffer);
         }

         else
         {
            throw new RuntimeException("Only Pin and Slider implemented right now");
         }

         Link link = joint.getLink();
         printLinkInformation(link, buffer);

         List<Joint> childrenJoints = joint.getChildrenJoints();

         for (Joint childJoint : childrenJoints)
         {
            buffer.append("Found Child Joint of " + jointName + ".\n");
            printJointInformation(childJoint, buffer);
         }

      }

      private void printPinJointInformation(OneDegreeOfFreedomJoint pinJoint, StringBuffer buffer)
      {
         buffer.append("Joint is a Pin Joint.\n");
         YoDouble q = pinJoint.getQYoVariable();
         buffer.append("Its q variable is named " + q.getName() + "\n");
      }

      private void printSliderJointInformation(SliderJoint sliderJoint, StringBuffer buffer)
      {
         buffer.append("Joint is a Slider Joint.\n");
         YoDouble q = sliderJoint.getQYoVariable();
         buffer.append("Its q variable is named " + q.getName() + "\n");
      }

      private void printFloatingJointInformation(FloatingJoint floatingJoint, StringBuffer buffer)
      {
         buffer.append("Joint is a Floating Joint.\n");
      }

      private void printFloatingPlanarJointInformation(FloatingPlanarJoint floatingPlanarJoint, StringBuffer buffer)
      {
         buffer.append("Joint is a Floating Planar Joint.\n");
      }

      private void printLinkInformation(Link link, StringBuffer buffer)
      {
         double mass = link.getMass();

         Vector3D comOffset = new Vector3D();
         link.getComOffset(comOffset);

         Matrix3D momentOfInertia = new Matrix3D();
         link.getMomentOfInertia(momentOfInertia);

         buffer.append("Mass = " + mass + "\n");
         buffer.append("comOffset = " + comOffset + "\n");
         buffer.append("momentOfInertia = \n" + momentOfInertia + "\n");
      }

      @Override
      public String toString()
      {
         StringBuffer buffer = new StringBuffer();
         getRobotInformationAsStringBuffer(buffer);
         return buffer.toString();
      }
   }
}
