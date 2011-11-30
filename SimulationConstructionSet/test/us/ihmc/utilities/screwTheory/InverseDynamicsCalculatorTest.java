package us.ihmc.utilities.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.LinkGraphics;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.util.robotExplorer.RobotExplorer;

/**
 * This currently needs to be here because it uses SCS classes to test the inverse dynamics calculator, and SCS isn't on the IHMCUtilities build path
 * @author Twan Koolen
 *
 */
public class InverseDynamicsCalculatorTest
{
   private static final boolean EXPLORE_AND_PRINT = false;
   
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);
   
   
   private final Random random = new Random(100L);

   @Before
   public void setUp()
   {
   }

   @Test
   public void testOneFreeRigidBody()
   {
      Robot robot = new Robot("robot");
      robot.setGravity(0.0);

      ReferenceFrame inertialFrame = ReferenceFrame.constructAWorldFrame("inertial");
      ReferenceFrame worldFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("world", inertialFrame, new Transform3D());
      RigidBody world = new RigidBody("world", worldFrame);

      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3d(), robot);
      robot.addRootJoint(rootJoint);

      SixDoFJoint rootInverseDynamicsJoint = new SixDoFJoint("root", world, worldFrame);

      Link link = createRandomLink("link", true);
      rootJoint.setLink(link);
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);

      RigidBody body = copyLinkAsRigidBody(link, rootInverseDynamicsJoint, "body");

      setRandomPosition(rootJoint, rootInverseDynamicsJoint);

      world.updateFramesRecursively();

      ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("rootExternalForcePoint", comOffset, robot);
      rootJoint.addExternalForcePoint(externalForcePoint);
      externalForcePoint.fx.set(random.nextDouble());
      externalForcePoint.fy.set(random.nextDouble());
      externalForcePoint.fz.set(random.nextDouble());

      Vector3d externalForce = new Vector3d();
      externalForcePoint.getForce(externalForce); // in world frame
      Transform3D worldToBody = worldFrame.getTransformToDesiredFrame(bodyFixedFrame);
      worldToBody.transform(externalForce);

      Wrench inputWrench = new Wrench(bodyFixedFrame, bodyFixedFrame, externalForce, new Vector3d());
      doRobotDynamics(robot);

      copyAccelerationFromForwardToInverse(rootJoint, rootInverseDynamicsJoint);

      HashMap<RigidBody, Wrench> externalWrenches = new HashMap<RigidBody, Wrench>(); // no external wrenches
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(world.getBodyFixedFrame(), inertialFrame, world.getBodyFixedFrame());
      ArrayList<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(inertialFrame, world, rootAcceleration, externalWrenches, jointsToIgnore, true, true);
      calculator.compute();

      Wrench outputWrench = new Wrench(null, null);
      rootInverseDynamicsJoint.packWrench(outputWrench);

      compareWrenches(inputWrench, outputWrench);
   }

   @Test
   public void testChainNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      HashMap<RevoluteJoint, PinJoint> jointMap = new HashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3d[] jointAxes = {X, Y, Z, X};
      
      double gravity = 0.0;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, true, true);
      
      InverseDynamicsCalculator calculator = createInverseDynamicsCalculator(elevator, gravity, worldFrame, true, true);
      calculator.compute();
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }
   
   
   @Test
   public void testTreeWithNoGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      HashMap<RevoluteJoint, PinJoint> jointMap = new HashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      double gravity = 0.0;

      int numberOfJoints = 3;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }
      
      InverseDynamicsCalculator calculator = createInverseDynamicsCalculator(elevator, gravity, worldFrame, true, true);
      calculator.compute();
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }
   
   @Test
   public void testTreeWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      HashMap<RevoluteJoint, PinJoint> jointMap = new HashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      double gravity = -9.8;

      int numberOfJoints = 100;
      createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity, true, true);

      if (EXPLORE_AND_PRINT)
      {
         exploreAndPrintRobot(robot);
         exploreAndPrintInverseDynamicsMechanism(elevator);
      }
      
      InverseDynamicsCalculator calculator = createInverseDynamicsCalculator(elevator, gravity, worldFrame, true, true);
      calculator.compute();
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertAccelerationsEqual(jointMap);
   }
   
   @Test
   public void testGravityCompensationForChain()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      HashMap<RevoluteJoint, PinJoint> jointMap = new HashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3d[] jointAxes = {X, Y, Z, X};
      double gravity = -9.8;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, false, false);
      
      InverseDynamicsCalculator calculator = createInverseDynamicsCalculator(elevator, gravity, worldFrame, false, false);
      calculator.compute();
      copyTorques(jointMap);
      doRobotDynamics(robot);
      assertZeroAccelerations(jointMap);
   }
   
   @Test
   public void testChainWithGravity()
   {
      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      HashMap<RevoluteJoint, PinJoint> jointMap = new HashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3d[] jointAxes = {X, Y, Z, X};
      
      double gravity = -9.8;
      createRandomChainRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, jointAxes, gravity, true, true);
      
      InverseDynamicsCalculator calculator = createInverseDynamicsCalculator(elevator, gravity, worldFrame, true, true);
      calculator.compute();
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
   
   private InverseDynamicsCalculator createInverseDynamicsCalculator(RigidBody elevator, double gravity, ReferenceFrame worldFrame, boolean doVelocityTerms, boolean doAcceleration)
   {
      HashMap<RigidBody, Wrench> externalWrenches = new HashMap<RigidBody, Wrench>(); // no external wrenches
      ReferenceFrame rootBodyFrame = elevator.getBodyFixedFrame();
      Vector3d linearAcceleration = new Vector3d(0.0, 0.0, -gravity);
      Vector3d angularAcceleration = new Vector3d();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBodyFrame, worldFrame, rootBodyFrame, linearAcceleration,
                                                      angularAcceleration);
      ArrayList<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsCalculator calculator = new InverseDynamicsCalculator(worldFrame, elevator, rootAcceleration, externalWrenches, jointsToIgnore, doVelocityTerms, doAcceleration);
      return calculator;
   }
   
   private void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-12; //3;
      JUnitTools.assertTuple3dEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }
   
   private void copyTorques(HashMap<RevoluteJoint, PinJoint> jointMap)
   {
      for (RevoluteJoint idJoint : jointMap.keySet())
      {
         PinJoint joint = jointMap.get(idJoint);
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
         PinJoint revoluteJoint = jointMap.get(idJoint);

         DoubleYoVariable qddVariable = revoluteJoint.getQDD();
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
      for (PinJoint joint : jointMap.values())
      {
         double qdd = joint.getQDD().getDoubleValue();
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

   private void createRandomChainRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBody elevator, Vector3d[] jointAxes, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration)
   {
      robot.setGravity(gravity);     

      RigidBody currentIDBody = elevator;
      PinJoint previousJoint = null;
      for (int i = 0; i < jointAxes.length; i++)
      {         
         Vector3d jointOffset = RandomTools.getRandomVector(random);
         Vector3d jointAxis = jointAxes[i];
         Matrix3d momentOfInertia = RandomTools.getRandomDiagonalMatrix3d(random);
         double mass = random.nextDouble();
         Vector3d comOffset = RandomTools.getRandomVector(random);
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
   
   
   private void createRandomTreeRobotAndSetJointPositionsAndVelocities(Robot robot, HashMap<RevoluteJoint, PinJoint> jointMap, ReferenceFrame worldFrame, RigidBody elevator, int numberOfJoints, double gravity, boolean useRandomVelocity, boolean useRandomAcceleration)
   {
      robot.setGravity(gravity);     
    
      ArrayList<PinJoint> potentialParentJoints = new ArrayList<PinJoint>();
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>(); // synchronized with potentialParentJoints

      
      for (int i = 0; i < numberOfJoints; i++)
      {         
         Vector3d jointOffset = RandomTools.getRandomVector(random);
         Vector3d jointAxis = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         jointAxis.normalize();
         Matrix3d momentOfInertia = RandomTools.getRandomDiagonalMatrix3d(random);
         double mass = random.nextDouble();
         Vector3d comOffset = RandomTools.getRandomVector(random);
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

      Vector3d comOffset = useZeroCoMOffset ? new Vector3d() : new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      link.setComOffset(comOffset);

      LinkGraphics linkGraphics = new LinkGraphics();
      linkGraphics.createInertiaEllipsoid(link, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   private void setRandomPosition(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      Point3d rootPosition = new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble());

      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();

      floatingJoint.setPosition(rootPosition);
      floatingJoint.setYawPitchRoll(yaw, pitch, roll);

      sixDoFJoint.setPosition(rootPosition);
      sixDoFJoint.setRotation(yaw, pitch, roll);
   }

   private void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint) // THIS IS WHERE THE PROBLEM IS. ACCELERATIONS DON'T WORK THIS WAY
   {
      ReferenceFrame frameBeforeJoint = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame frameAfterJoint = sixDoFJoint.getFrameAfterJoint();

      double linearAccelerationX = floatingJoint.getQddx().getDoubleValue();
      double linearAccelerationY = floatingJoint.getQddy().getDoubleValue();
      double linearAccelerationZ = floatingJoint.getQddz().getDoubleValue();

      FrameVector linearAcceleration = new FrameVector(frameBeforeJoint, linearAccelerationX, linearAccelerationY, linearAccelerationZ);
      linearAcceleration = linearAcceleration.changeFrameCopy(frameAfterJoint);

      double angularAccelerationX = floatingJoint.getAngularAccelerationX().getDoubleValue();
      double angularAccelerationY = floatingJoint.getAngularAccelerationY().getDoubleValue();
      double angularAccelerationZ = floatingJoint.getAngularAccelerationZ().getDoubleValue();
      FrameVector angularAcceleration = new FrameVector(frameAfterJoint, angularAccelerationX, angularAccelerationY, angularAccelerationZ);

      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector(frameAfterJoint, frameBeforeJoint, frameAfterJoint,
                                                       linearAcceleration.getVector(), angularAcceleration.getVector());

      sixDoFJoint.setDesiredAcceleration(jointAcceleration);
   }

   private static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      Transform3D transformToParent = new Transform3D();
      transformToParent.set(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }
}
