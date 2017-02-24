package us.ihmc.avatar.manipulating;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCManipulatingReachingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCBehaviorTestHelper drcTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   private boolean isKinematicsToolboxVisualizerEnabled = false;

   WayPointGeneratorWOCollision wayPointGenerator;
   GhostRobotArm ghostRobotArm;
   CollisionConstraintCondition collisionConstraintCondition;
   SimpleCollisionDetector collisionDetector;

   private final SideDependentList<GeometricJacobian> workArmJacobians = new SideDependentList<GeometricJacobian>();

   private Point3d targetPosition = new Point3d(0.8, -0.3, 0);
   private double targetRadius = 0.05;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()) // set
      // do
      // not
      // auto
      // quit
      // if(simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be
      // recycled.
      if (drcTestHelper != null)
      {
         drcTestHelper.destroySimulation();
         drcTestHelper = null;
      }
      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.close();
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public class ManipulatingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D EnvSet;

      public ManipulatingEnvironment()
      {
         EnvSet = DefaultCommonAvatarEnvironment.setUpGround("Ground");
         
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         // TODO Auto-generated method stub
         return EnvSet;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
         // TODO Auto-generated method stub
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 150000)
   public void testDoit() throws SimulationExceededMaximumTimeException, IOException
   {

      testRRT();
      // testCollisionForGivenPoint();
      // testSingleHandTrajectory();
      // testReachingWithWayPoints();
      // testReaching();

   }

   private void testRRT() throws SimulationExceededMaximumTimeException, IOException
   {
      CommonAvatarEnvironmentInterface Env = new ManipulatingEnvironment(); // all-flat

      PrintTools.info("RRT Test");
      String name = "RRTTest";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcTestHelper = new DRCBehaviorTestHelper(Env, name, selectedLocation, simulationTestingParameters, getRobotModel());
      
      FullHumanoidRobotModel fullRobotModel = drcTestHelper.getControllerFullRobotModel();
      
      SimulationConstructionSet scs = drcTestHelper.getSimulationConstructionSet();

      boolean success = drcTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
                   
      
      
      
      
      
      
      
       
      PrintTools.info("Finished !");

      success = drcTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      // end
      ThreadTools.sleep(100);
      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

   }
   
   private void testCollisionForGivenPoint() throws SimulationExceededMaximumTimeException, IOException
   {
      CommonAvatarEnvironmentInterface Env = new ManipulatingEnvironment(); // all-flat

      PrintTools.info("WayPointsTest");
      String name = "WayPointsTest";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcTestHelper = new DRCBehaviorTestHelper(Env, name, selectedLocation, simulationTestingParameters, getRobotModel());
      
      FullHumanoidRobotModel fullRobotModel = drcTestHelper.getControllerFullRobotModel();
      
      SimulationConstructionSet scs = drcTestHelper.getSimulationConstructionSet();

      boolean success = drcTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
                  
      ghostRobotArm = new GhostRobotArm(drcTestHelper);            
      collisionConstraintCondition = new CollisionConstraintCondition(ghostRobotArm);      
      
      ArrayList<Graphics3DObject> envShow = new ArrayList<Graphics3DObject>();
      envShow = collisionConstraintCondition.getEnvironment();
      scs.addStaticLinkGraphics(envShow);
      
      drcTestHelper.simulateAndBlockAndCatchExceptions(1.5-getRobotModel().getControllerDT());
      assertTrue(success);
      
      wayPointGenerator = new WayPointGeneratorWOCollision(collisionConstraintCondition);
      
      drcTestHelper.updateRobotModel();      
      
      Pose initialPose = new Pose();      
      Quat4d initialOrientation = new Quat4d();
      
      //RigidBodyTransform initialTransform = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame().getTransformToWorldFrame();
      RigidBodyTransform initialTransform = fullRobotModel.getHandControlFrame(RobotSide.RIGHT).getTransformToWorldFrame();
      
      
      RotationTools.convertTransformToQuaternion(initialTransform, initialOrientation);      
      
      initialPose.setPosition(initialTransform.mat03, initialTransform.mat13, initialTransform.mat23);
      initialPose.setOrientation(initialOrientation);
            
      Point3d finalPosition = new Point3d(targetPosition.x-targetRadius, targetPosition.y, 1.0);
      Quat4d finalOrientation = new Quat4d();
        
      wayPointGenerator.setInitialPose(initialPose);

      RotationTools.convertYawPitchRollToQuaternion(0.0 * Math.PI, 0.0 * Math.PI, -0.5 * Math.PI, finalOrientation);
      Pose finalPose = new Pose(finalPosition, finalOrientation);
      wayPointGenerator.setFinalPose(finalPose);

      wayPointGenerator.setNumberOfWayPoints(10);
      wayPointGenerator.initializeWayPoints();
      

      System.out.println("====================================");
      System.out.println("Initial Config");
      System.out.printf("%.3f %.3f %.3f \n", wayPointGenerator.initialPoint.getX(), wayPointGenerator.initialPoint.getY(),
                        wayPointGenerator.initialPoint.getZ());
      System.out.printf("%.3f %.3f %.3f %.3f \n", wayPointGenerator.initialPoint.getOrientation().w, wayPointGenerator.initialPoint.getOrientation().x,
                        wayPointGenerator.initialPoint.getOrientation().y, wayPointGenerator.initialPoint.getOrientation().z);

      System.out.println("Goal Config");
      System.out.printf("%.3f %.3f %.3f \n", wayPointGenerator.finalPoint.getX(), wayPointGenerator.finalPoint.getY(),
                        wayPointGenerator.finalPoint.getZ());
      System.out.printf("%.3f %.3f %.3f %.3f \n", wayPointGenerator.finalPoint.getOrientation().w, wayPointGenerator.finalPoint.getOrientation().x,
                        wayPointGenerator.finalPoint.getOrientation().y, wayPointGenerator.finalPoint.getOrientation().z);
      System.out.println("====================================");


      
      

      
      
      wayPointGenerator.generateWayPoints();
      
      for (int i = 0; i < wayPointGenerator.getNumberOfWayPoints(); i++)
      {
         // Way Points Position
         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(wayPointGenerator.getWayPointPosition(i));
         sphere.addSphere(0.015, new YoAppearanceRGBColor(Color.BLACK, 0.0));
         scs.addStaticLinkGraphics(sphere);

         // Way Points Orientation
         ArrayList<Graphics3DObject> axis = new ArrayList<Graphics3DObject>();
         axis = wayPointGenerator.createXYZAxisOfWayPoint(i);

         scs.addStaticLinkGraphics(axis);
         
//          RigidBodyTransform temp = new RigidBodyTransform();
//          temp.setTranslation(wayPointGenerator.getWayPointPosition(i));
//          temp.setRotation(wayPointGenerator.getWayPointOrientation(i));
//          ghostRobotArm.updateJointOfGhost(temp);
//          
//          collisionConstraintCondition.updateCurrentState();
//          boolean colResult = collisionConstraintCondition.getResult();
//          
//          ArrayList<Graphics3DObject> robotArm = new ArrayList<Graphics3DObject>();
//          robotArm = ghostRobotArm.getGhostCollisionModel();
//          scs.addStaticLinkGraphics(robotArm);
            
          drcTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      }

      


      
  
      
      
      
      
      
//      RigidBodyTransform temp = new RigidBodyTransform();
//      temp.setTranslation(wayPointGenerator.getWayPointPosition(3));
//      temp.setRotation(wayPointGenerator.getWayPointOrientation(3));      
//      ghostRobotArm.updateJointOfGhost(temp);
//      
//      collisionConstraintCondition.updateCurrentState();
//      boolean colResult = collisionConstraintCondition.getResult();
//      PrintTools.info("result!! "+colResult);
      
      
      
//      ArrayList<Graphics3DObject> jointAxis = new ArrayList<Graphics3DObject>();
//      jointAxis = ghostRobotArm.createXYZAxis(ghostRobotArm.getTransformOfSHX());
//      scs.addStaticLinkGraphics(jointAxis);
//      jointAxis = ghostRobotArm.createXYZAxis(ghostRobotArm.getTransformOfEBX());
//      scs.addStaticLinkGraphics(jointAxis);
//      jointAxis = ghostRobotArm.createXYZAxis(ghostRobotArm.getTransformOfWRX());
//      scs.addStaticLinkGraphics(jointAxis);
//      jointAxis = ghostRobotArm.createXYZAxis(ghostRobotArm.getTransformOfEND());
//      scs.addStaticLinkGraphics(jointAxis);

//      jointAxis = ghostRobotArm.createXYZAxis(collisionConstraintCondition.getCenterTransformOfTwo(ghostRobotArm.getTransformOfSHX(), ghostRobotArm.getTransformOfEBX()));
//      scs.addStaticLinkGraphics(jointAxis);
//      jointAxis = ghostRobotArm.createXYZAxis(collisionConstraintCondition.getCenterTransformOfTwo(ghostRobotArm.getTransformOfEBX(), ghostRobotArm.getTransformOfWRX()));
//      scs.addStaticLinkGraphics(jointAxis);    
//      jointAxis = ghostRobotArm.createXYZAxis(collisionConstraintCondition.getCenterTransformOfTwo(ghostRobotArm.getTransformOfWRX(), ghostRobotArm.getTransformOfEND()));
//      scs.addStaticLinkGraphics(jointAxis);
      
      
      
      
//      RigidBodyTransform capsuleCenter = new RigidBodyTransform();
//      collisionConstraintCondition.collisionDetector.collisionObjects.get(2).getTransformToWorld(capsuleCenter);
//      jointAxis = ghostRobotArm.createXYZAxis(capsuleCenter);
//      scs.addStaticLinkGraphics(jointAxis);
//      ghostRobotArm.printRigidBodyTR(capsuleCenter);
                  
//      for (int i = 0; i < ghostRobotArm.jointOfGhost.length; i++)
//      {         
//         double jointPosition = ghostRobotArm.jointOfGhost[i].getQ();
//
//         Joint scsJoint = drcTestHelper.getRobot().getJoint(ghostRobotArm.jointOfOrigin[i].getName());
//         if (scsJoint instanceof PinJoint)
//         {
//            PinJoint pinJoint = (PinJoint) scsJoint;
//            pinJoint.setQ(jointPosition);
//         }
//         else
//         {
//            PrintTools.info(ghostRobotArm.jointOfGhost[i].getName() + " was not a PinJoint.");
//         }
//      }

//      ArrayList<Graphics3DObject> robotArm = new ArrayList<Graphics3DObject>();
//      robotArm = ghostRobotArm.getGhostCollisionModel();
//      scs.addStaticLinkGraphics(robotArm);

      for (int i =0;i<250;i++)
      {
         ArrayList<Graphics3DObject> randomAxis = new ArrayList<Graphics3DObject>();
         randomAxis = wayPointGenerator.createRandomXYZAxis();
         scs.addStaticLinkGraphics(randomAxis);
      }
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.translate(1.0, 0.0, 2.0);
      sphere.addSphere(0.0075, new YoAppearanceRGBColor(Color.BLACK, 0.0));
      scs.addStaticLinkGraphics(sphere);
      
         
      
      PrintTools.info("Finished !");

      //      success = drcTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      //      assertTrue(success);
      //
      //      // end
      //      ThreadTools.sleep(100);
      //      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      //      assertTrue(success);

   }

   private void testSingleHandTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      CommonAvatarEnvironmentInterface Env = new ManipulatingEnvironment(); // all-flat
      // +
      // stuff
      // env

      PrintTools.info("SingleGoal");
      String name = "SingleGoal";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcTestHelper = new DRCBehaviorTestHelper(Env, name, selectedLocation, simulationTestingParameters, getRobotModel());

      boolean success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      FullHumanoidRobotModel fullRobotModel = drcTestHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = drcTestHelper.getSimulationConstructionSet();

      RobotSide robotSide = RobotSide.RIGHT;

      double trajectoryTime = 3.0;
      Point3d desiredPosition = new Point3d(0.65, -0.3, 1.0);
      Quat4d desiredOrientation = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(0.0, 0.0, -1.5, desiredOrientation);
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition,
                                                                              desiredOrientation);

      drcTestHelper.send(handTrajectoryMessage);

      success = drcTestHelper.simulateAndBlockAndCatchExceptions(3.0 + trajectoryTime);
      assertTrue(success);

      // end
      ThreadTools.sleep(100);
      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   }

   private void testReachingWithWayPoints() throws SimulationExceededMaximumTimeException, IOException
   {
      CommonAvatarEnvironmentInterface Env = new ManipulatingEnvironment(); // all-flat
      // +
      // stuff
      // env

      PrintTools.info("JointSpaceTest");
      String name = "JointSpaceTest";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcTestHelper = new DRCBehaviorTestHelper(Env, name, selectedLocation, simulationTestingParameters, getRobotModel());

      boolean success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      FullHumanoidRobotModel fullRobotModel = drcTestHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = drcTestHelper.getSimulationConstructionSet();

      double trajectoryTime = 2.0;
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(RobotSide.RIGHT);
      OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);
      double[] desiredJointVelocities = new double[numberOfJoints];
      double[] desiredJointPositions = new double[armJoints.length];
      ArmTrajectoryMessage armTrajectoryMessage;

      PrintTools.info("1st " + armJoints.length);

      desiredJointPositions[0] = 0.6;
      desiredJointPositions[1] = 1.1;
      desiredJointPositions[2] = 1.2;
      desiredJointPositions[3] = -1.5;
      desiredJointPositions[4] = 1.2;
      desiredJointPositions[5] = -0.4;
      desiredJointPositions[6] = 0.0;

      armTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, trajectoryTime, desiredJointPositions);

      drcTestHelper.send(armTrajectoryMessage);

      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime * 2);
      assertTrue(success);

      // end
      ThreadTools.sleep(100);
      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

   }

   private void testReaching() throws SimulationExceededMaximumTimeException, IOException
   {
      CommonAvatarEnvironmentInterface Env = new ManipulatingEnvironment(); // all-flat
      // +
      // stuff
      // env

      String name = "WholeBody Reaching Test";
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcTestHelper = new DRCBehaviorTestHelper(Env, name, selectedLocation, simulationTestingParameters, getRobotModel());

      kinematicsToolboxModule = new KinematicsToolboxModule(getRobotModel(), isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                           PacketDestination.KINEMATICS_TOOLBOX_MODULE);

      boolean success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      SimulationConstructionSet scs = drcTestHelper.getSimulationConstructionSet();
      drcTestHelper.updateRobotModel();
      PrintTools.info("WholeBody Reaching Test");

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcTestHelper.getYoTime(),
                                                                                     drcTestHelper.getBehaviorCommunicationBridge(),
                                                                                     getRobotModel().createFullRobotModel());

      ReferenceFrame handControlFrameR = drcTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = drcTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      FramePose curHandPoseR = new FramePose(handControlFrameR);
      FramePose curHandPoseL = new FramePose(handControlFrameL);
      curHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      curHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());

      Point3d curHandPosR = new Point3d();
      Point3d curHandPosL = new Point3d();
      Quat4d curHandOriR = new Quat4d();
      Quat4d curHandOriL = new Quat4d();
      curHandPoseR.getPosition(curHandPosR);
      curHandPoseL.getPosition(curHandPosL);
      curHandPoseR.getOrientation(curHandOriR);
      curHandPoseL.getOrientation(curHandOriL);

      FramePose desHandPoseR = new FramePose(handControlFrameR);
      FramePose desHandPoseL = new FramePose(handControlFrameL);
      desHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      desHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());

      Point3d desHandPosR = new Point3d(0.7, 0.1, 0.4);
      Point3d desHandPosL = new Point3d(-0.0, 0.0, 0.0);
      Quat4d desHandOriR = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(0.0, -0.3, 1.0, desHandOriR);
      desHandPoseR.translate(desHandPosR);
      desHandPoseL.translate(desHandPosL);
      desHandPoseR.setOrientation(desHandOriR);

      ik.setTrajectoryTime(1.0);
      ik.setDesiredHandPose(RobotSide.RIGHT, desHandPoseR);
      ik.setDesiredHandPose(RobotSide.LEFT, desHandPoseL);

      //drcTestHelper.dispatchBehavior(ik);

      curHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      curHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      curHandPoseR.getPosition(curHandPosR);
      curHandPoseL.getPosition(curHandPosL);
      curHandPoseR.getOrientation(curHandOriR);
      curHandPoseL.getOrientation(curHandOriL);
      
      PrintTools.info("Init Pos is "+ curHandPosR.x + " " + curHandPosR.y + " " + curHandPosR.z);
      PrintTools.info("Init Ori is "+ curHandOriR.x + " " + curHandOriR.y + " " + curHandOriR.z + " " + curHandOriR.w);

      // end
      ThreadTools.sleep(100);
      success = drcTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

   }

}
