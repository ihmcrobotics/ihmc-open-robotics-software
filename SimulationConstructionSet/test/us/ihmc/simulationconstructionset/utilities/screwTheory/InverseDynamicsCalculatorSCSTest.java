package us.ihmc.simulationconstructionset.utilities.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsMechanismExplorer;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.robotExplorer.RobotExplorer;

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

   @Before
   public void setUp()
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOneFreeRigidBody()
   {
      Robot robot = new Robot("robot");
      double gravity = -9.81;
      robot.setGravity(gravity);

      ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", inertialFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3D(0.1, 0.2, 0.3), robot);
      robot.addRootJoint(rootJoint);

      SixDoFJoint rootInverseDynamicsJoint = new SixDoFJoint("root", elevator, elevatorFrame);

      Link link = createRandomLink("link", false);
      rootJoint.setLink(link);
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
//      System.out.println("comOffset = " + comOffset);

      RigidBody body = copyLinkAsRigidBody(link, rootInverseDynamicsJoint, "body");

      setRandomPosition(rootJoint, rootInverseDynamicsJoint);
      elevator.updateFramesRecursively();

      setRandomVelocity(rootJoint, rootInverseDynamicsJoint);

      ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      
      TranslationReferenceFrame forceApplicationFrame = new TranslationReferenceFrame("forceApplicationFrame", bodyFixedFrame);
      Vector3D translationFromCoM = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      forceApplicationFrame.updateTranslation(translationFromCoM);
      
      Vector3D externalForcePointOffset = new Vector3D(comOffset);
      externalForcePointOffset.add(translationFromCoM);
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("rootExternalForcePoint", externalForcePointOffset, robot.getRobotsYoVariableRegistry());
      rootJoint.addExternalForcePoint(externalForcePoint);
      externalForcePoint.setForce(random.nextDouble(), random.nextDouble(), random.nextDouble());

      Vector3D externalForce = new Vector3D();
      externalForcePoint.getForce(externalForce); // in world frame
      RigidBodyTransform worldToBody = elevatorFrame.getTransformToDesiredFrame(forceApplicationFrame);
      worldToBody.transform(externalForce);

      Wrench inputWrench = new Wrench(forceApplicationFrame, forceApplicationFrame, externalForce, new Vector3D());
      doRobotDynamics(robot);

      copyAccelerationFromForwardToInverse(rootJoint, rootInverseDynamicsJoint);

      TwistCalculator twistCalculator = new TwistCalculator(inertialFrame, elevator);
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -gravity);
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();

      Wrench outputWrench = new Wrench(null, null);
      rootInverseDynamicsJoint.getWrench(outputWrench);
      
      outputWrench.changeBodyFrameAttachedToSameBody(forceApplicationFrame);
      outputWrench.changeFrame(forceApplicationFrame);

      compareWrenches(inputWrench, outputWrench);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testChainNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3D[] jointAxes = {X, Y, Z, X};
      
      double gravity = 0.0;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, true, true, random);
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, true, true);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTreeWithNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testTreeWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void testDoingInverseDynamicsTermPerTerm()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      LinkedHashMap<RevoluteJoint, Double> tau_gravity = new LinkedHashMap<RevoluteJoint, Double>();
      LinkedHashMap<RevoluteJoint, Double> tau_cc = new LinkedHashMap<RevoluteJoint, Double>();
      LinkedHashMap<RevoluteJoint, Double> tau_qdd = new LinkedHashMap<RevoluteJoint, Double>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testDoingNothing()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGravityCompensationForChain()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3D[] jointAxes = {X, Y, Z, X};
      double gravity = -9.8;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, false, false, random);
      
      createInverseDynamicsCalculatorAndCompute(elevator, gravity, worldFrame, false, false);
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertZeroAccelerations(jointMap);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testChainWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
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
   
   private void exploreAndPrintInverseDynamicsMechanism(RigidBody elevator)
   {
      InverseDynamicsMechanismExplorer idMechanismExplorer = new InverseDynamicsMechanismExplorer(elevator);
      System.out.println("-----------------------------");
      System.out.println(idMechanismExplorer);
      System.out.println("-----------------------------");
   }
   
   private InverseDynamicsCalculator createInverseDynamicsCalculatorAndCompute(RigidBody elevator, double gravity, ReferenceFrame worldFrame, boolean doVelocityTerms, boolean doAcceleration)
   {
      TwistCalculator twistCalculator = new TwistCalculator(worldFrame, elevator);
      ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();
      SpatialAccelerationVector rootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), -gravity);
      LinkedHashMap<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
      ArrayList<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(inertialFrame, rootAcceleration, externalWrenches, jointsToIgnore, doVelocityTerms, doAcceleration, twistCalculator);
//      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -gravity);
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
      return inverseDynamicsCalculator;
   }
   
   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-12; //3;
      EuclidCoreTestTools.assertTuple3DEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
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

         DoubleYoVariable qddVariable = revoluteJoint.getQDDYoVariable();
         double qdd = qddVariable.getDoubleValue();
         double qddInverse = idJoint.getQddDesired();

//         DoubleYoVariable tauVariable = revoluteJoint.getTau();
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

   private static void createRandomChainRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBody elevator, Vector3D[] jointAxes, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration, Random random)
   {
      robot.setGravity(gravity);     

      RigidBody currentIDBody = elevator;
      PinJoint previousJoint = null;
      for (int i = 0; i < jointAxes.length; i++)
      {         
         Vector3D jointOffset = RandomTools.generateRandomVector(random);
         Vector3D jointAxis = jointAxes[i];
         Matrix3D momentOfInertia = RandomTools.generateRandomDiagonalMatrix3d(random);
         double mass = random.nextDouble();
         Vector3D comOffset = RandomTools.generateRandomVector(random);
         double jointPosition = random.nextDouble();
         double jointVelocity = useRandomVelocity ? random.nextDouble() : 0.0;
         double jointAcceleration = useRandomAcceleration ? random.nextDouble() : 0.0;
         
         RevoluteJoint currentIDJoint = ScrewTools.addRevoluteJoint("jointID" + i, currentIDBody, jointOffset, jointAxis);
         currentIDJoint.setQ(jointPosition);
         currentIDJoint.setQd(jointVelocity);
         currentIDJoint.setQddDesired(jointAcceleration);
         
         currentIDBody = ScrewTools.addRigidBody("bodyID" + i, currentIDJoint, momentOfInertia, mass, comOffset);
         
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
   
   
   public static void createRandomTreeRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBody elevator, int numberOfJoints, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration, Random random)
   {
      robot.setGravity(gravity);     
    
      ArrayList<PinJoint> potentialParentJoints = new ArrayList<PinJoint>();
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>(); // synchronized with potentialParentJoints

      
      for (int i = 0; i < numberOfJoints; i++)
      {         
         Vector3D jointOffset = RandomTools.generateRandomVector(random);
         Vector3D jointAxis = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         jointAxis.normalize();
         Matrix3D momentOfInertia = RandomTools.generateRandomDiagonalMatrix3d(random);
         double mass = random.nextDouble();
         Vector3D comOffset = RandomTools.generateRandomVector(random);
         double jointPosition = random.nextDouble();
         double jointVelocity = useRandomVelocity ? random.nextDouble() : 0.0;
         double jointAcceleration = useRandomAcceleration ? random.nextDouble() : 0.0;

         PinJoint currentJoint = new PinJoint("joint" + i, jointOffset, robot, jointAxis);
         currentJoint.setInitialState(jointPosition, jointVelocity);
         RigidBody inverseDynamicsParentBody;
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

         RevoluteJoint currentIDJoint = ScrewTools.addRevoluteJoint("jointID" + i, inverseDynamicsParentBody, jointOffset, jointAxis);
         currentIDJoint.setQ(jointPosition);
         currentIDJoint.setQd(jointVelocity);
         currentIDJoint.setQddDesired(jointAcceleration);
         ScrewTools.addRigidBody("bodyID" + i, currentIDJoint, momentOfInertia, mass, comOffset);

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

   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   private void setRandomPosition(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Point3D rootPosition = new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();

      floatingJoint.setPosition(rootPosition);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

      sixDoFJoint.setPosition(rootPosition);
      sixDoFJoint.setRotation(yaw, pitch, roll);
   }
   
   private final FrameVector linearVelocityFrameVector = new FrameVector();
   private final FrameVector angularVelocityFrameVector = new FrameVector();
   
   private void setRandomVelocity(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Vector3D linearVelocity = RandomTools.generateRandomVector(random, 1.0);
      Vector3D angularVelocity = RandomTools.generateRandomVector(random, 1.0);

      floatingJoint.setVelocity(linearVelocity);
      floatingJoint.setAngularVelocityInBody(angularVelocity);

      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      floatingJoint.getVelocity(linearVelocityFrameVector);
      linearVelocityFrameVector.changeFrame(bodyFrame);

      floatingJoint.getAngularVelocity(angularVelocityFrameVector, bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, linearVelocityFrameVector.getVector(), angularVelocityFrameVector.getVector());
      sixDoFJoint.setJointTwist(bodyTwist);
   }

   
   private void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      // TODO: Get this to work when the FloatingJoint has an offset.

      Twist bodyTwist = new Twist();
      sixDoFJoint.getJointTwist(bodyTwist);

      FrameVector originAcceleration = new FrameVector(sixDoFJoint.getFrameBeforeJoint());
      FrameVector angularAcceleration = new FrameVector(sixDoFJoint.getFrameAfterJoint());

      floatingJoint.getLinearAccelerationInWorld(originAcceleration.getVector());
      floatingJoint.getAngularAccelerationInBody(angularAcceleration.getVector());
      originAcceleration.changeFrame(sixDoFJoint.getFrameBeforeJoint());

      SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector(sixDoFJoint.getFrameAfterJoint(), sixDoFJoint.getFrameBeforeJoint(), sixDoFJoint.getFrameAfterJoint());
      
      spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
   }

   private static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3D offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }
}
