package us.ihmc.avatar.behaviorTests.wholeBodyPlanningTest;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.WholeBodyTrajectoryToolboxController;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.WholeBodyTrajectoryToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SetBooleanParameterPacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WayPointsByVRUIBehaviorStateMachine;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WayPointsByVRUIBehaviorStateMachine.WayPointsByVRUIBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.WholeBodyTrajectoryToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DualWayPointsPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WayPointsPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DualWayPointsTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.WayPointsTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarCWBPlannerForVRUITest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private static boolean visulaizerOn = true;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   
   private WholeBodyTrajectoryToolboxModule cwbPlanningToolboxModule;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator kinematicsToolboxCommunicator;
   private PacketCommunicator wholeBodyTrajectoryToolboxCommunicator;

   private void setupCWBPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      cwbPlanningToolboxModule = new WholeBodyTrajectoryToolboxModule(robotModel, fullRobotModel, null, visulaizerOn);
      kinematicsToolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      wholeBodyTrajectoryToolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE);
   }

   private WayPointsPacket wayPointsPacket;
   
   private void setupWayPointsPacket()
   {
      // for left hand
      Pose3D[] poseOfWayPoints = new Pose3D[4];
      Quaternion wayPointOrientation;

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, 0.4, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(-Math.PI * 0.2);
      wayPointOrientation.appendYawRotation(-Math.PI * 0.3);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6, 0.1, 1.5), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(-Math.PI * 0.2);
      poseOfWayPoints[2] = new Pose3D(new Point3D(0.5, -0.2, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[3] = new Pose3D(new Point3D(0.6, 0.4, 0.8), wayPointOrientation);
      
      wayPointsPacket = new WayPointsPacket();
      wayPointsPacket.setRobotSide(RobotSide.LEFT);
      wayPointsPacket.setWayPoints(poseOfWayPoints, 10.0);
      
      // for right hand
      poseOfWayPoints = new Pose3D[5];      

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, -0.4, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(-Math.PI * 0.2);
      wayPointOrientation.appendYawRotation(Math.PI * 0.1);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6, -0.1, 1.5), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(Math.PI * 0.2);
      poseOfWayPoints[2] = new Pose3D(new Point3D(0.5, 0.2, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.1);
      wayPointOrientation.appendYawRotation(Math.PI * 0.2);
      poseOfWayPoints[3] = new Pose3D(new Point3D(0.6, -0.4, 0.8), wayPointOrientation);
      
      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(-Math.PI * 0.1);
      poseOfWayPoints[4] = new Pose3D(new Point3D(0.6, -0.5, 1.0), wayPointOrientation);
      
      wayPointsPacket = new WayPointsPacket();
      wayPointsPacket.setRobotSide(RobotSide.RIGHT);
      wayPointsPacket.setWayPoints(poseOfWayPoints, 10.0);
   }
   
   private DualWayPointsPacket dualWayPointsPacket;
   
   private void setupDualWayPointsPacket()
   {
      dualWayPointsPacket = new DualWayPointsPacket();
      dualWayPointsPacket.setTrajectoryTime(10.0);
      
      // for left hand
      Pose3D[] poseOfWayPoints = new Pose3D[2];
      Quaternion wayPointOrientation;

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.1);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, 0.4, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(-Math.PI * 0.2);
      wayPointOrientation.appendYawRotation(-Math.PI * 0.3);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6, 0.2, 1.5), wayPointOrientation);
      
      dualWayPointsPacket.setWayPoints(RobotSide.LEFT, poseOfWayPoints);
      
      // for right hand
      poseOfWayPoints = new Pose3D[3];      

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.1);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, -0.4, 1.0), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(-Math.PI * 0.2);
      wayPointOrientation.appendYawRotation(Math.PI * 0.1);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6, -0.2, 1.5), wayPointOrientation);

      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(Math.PI * 0.2);
      poseOfWayPoints[2] = new Pose3D(new Point3D(0.5, -0.2, 1.2), wayPointOrientation);
      
      dualWayPointsPacket.setWayPoints(RobotSide.RIGHT, poseOfWayPoints);
   }
   
   public ArrayList<Graphics3DObject> getXYZAxis(Pose3D pose, double length)
   {
      double axisHeight = length;
      double axisRadius = length / 10.0;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Point3D centerPoint = new Point3D(pose.getPosition());

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);

      RotationMatrix axisOrientation = new RotationMatrix(pose.getOrientation());

      RotationMatrix axisX = new RotationMatrix(axisOrientation);
      RotationMatrix axisY = new RotationMatrix(axisOrientation);
      RotationMatrix axisZ = new RotationMatrix(axisOrientation);

      retZ.rotate(axisZ);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      axisX.appendPitchRotation(Math.PI * 0.5);
      retX.rotate(axisX);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      axisY.appendRollRotation(-Math.PI * 0.5);
      retY.rotate(axisY);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (true)
      //if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (cwbPlanningToolboxModule != null)
      {
         cwbPlanningToolboxModule.destroy();
         cwbPlanningToolboxModule = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (kinematicsToolboxCommunicator != null)
      {
         kinematicsToolboxCommunicator.closeConnection();
         kinematicsToolboxCommunicator = null;
      }

      if (wholeBodyTrajectoryToolboxCommunicator != null)
      {
         wholeBodyTrajectoryToolboxCommunicator.closeConnection();
         wholeBodyTrajectoryToolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, getRobotModel());

      setupCWBPlanningToolboxModule();
   }
   
   public void testForWayPointsTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      setupWayPointsPacket();      
      
      WayPointsTrajectory endeffectorTrajectory = new WayPointsTrajectory(wayPointsPacket);

      scs.addStaticLinkGraphics(getXYZAxis(wayPointsPacket.getPoseOfWayPoint(0), 0.05));
      scs.addStaticLinkGraphics(getXYZAxis(wayPointsPacket.getPoseOfWayPoint(1), 0.05));
      scs.addStaticLinkGraphics(getXYZAxis(wayPointsPacket.getPoseOfWayPoint(2), 0.05));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(1.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(4.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(6.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(8.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(9.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(0.0, RobotSide.RIGHT, new ConfigurationSpace()), 0.2));
      
      // setupDualWayPointsPacket();      
      // DualWayPointsTrajectory endeffectorTrajectory = new DualWayPointsTrajectory(dualWayPointsPacket);      
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }
   
   public void testForReachability() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
      
      drcBehaviorTestHelper.updateRobotModel();
      WholeBodyInverseKinematicsBehavior wholebodyBehavior = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                                    drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                                    drcBehaviorTestHelper.getSDFFullRobotModel());

      FramePose handFramePose = new FramePose(ReferenceFrame.getWorldFrame());
      handFramePose.setPosition(0.6, -0.4, 1.0);

      wholebodyBehavior.setTrajectoryTime(3.0);
      wholebodyBehavior.setDesiredHandPose(RobotSide.RIGHT, handFramePose);

      drcBehaviorTestHelper.updateRobotModel();
      drcBehaviorTestHelper.dispatchBehavior(wholebodyBehavior);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      System.out.println("End");
   }

   public void testForReachabilityEETraj() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
            
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);



      setupWayPointsPacket();
      
      ConstrainedEndEffectorTrajectory endeffectorTrajectory = new WayPointsTrajectory(wayPointsPacket);
      
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
      
      drcBehaviorTestHelper.updateRobotModel();
      WholeBodyInverseKinematicsBehavior wholebodyBehavior = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                                    drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                                    drcBehaviorTestHelper.getSDFFullRobotModel());

      for(RobotSide robotSide:RobotSide.values)
      {
         Pose3D desiredPose = endeffectorTrajectory.getEndEffectorPose(6.0, robotSide, new ConfigurationSpace());

         desiredPose.appendTranslation(-0.0, 0.0, 0.0);
         
         FramePose handFramePose = new FramePose(ReferenceFrame.getWorldFrame(), desiredPose);

         wholebodyBehavior.setTrajectoryTime(3.0);
         wholebodyBehavior.setDesiredHandPose(robotSide, handFramePose);
         
         scs.addStaticLinkGraphics(getXYZAxis(desiredPose, 0.1));
      }
      
      drcBehaviorTestHelper.updateRobotModel();
      drcBehaviorTestHelper.dispatchBehavior(wholebodyBehavior);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      System.out.println("End");
   }
   
   public void testForBehavior() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      /*
       * Behavior create.
       */
      WayPointsByVRUIBehaviorStateMachine wayPointsByVRUIBehaviorStateMachine = new WayPointsByVRUIBehaviorStateMachine(getRobotModel(),
                                                                                                                        drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                                                        drcBehaviorTestHelper.getYoTime(),
                                                                                                                        sdfFullRobotModel, referenceFrames);

      referenceFrames.updateFrames();

      System.out.println("Behavior Dispatch");
      drcBehaviorTestHelper.dispatchBehavior(wayPointsByVRUIBehaviorStateMachine);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      
      System.out.println("WayPointsPacket Dispatch");
      
      setupWayPointsPacket();      
      setupDualWayPointsPacket();      
      
      // drcBehaviorTestHelper.getBehaviorCommunicationBridge().sendPacketToBehavior(wayPointsPacket);
      drcBehaviorTestHelper.getBehaviorCommunicationBridge().sendPacketToBehavior(dualWayPointsPacket);
      System.out.println("WayPointsPacket Dispatch done");

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      double yoTime = 0.0;
      while (yoTime < 100)
      {
         drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         yoTime = drcBehaviorTestHelper.getYoTime().getDoubleValue();
         if (wayPointsByVRUIBehaviorStateMachine.getStateMachine().getCurrentStateEnum() == WayPointsByVRUIBehaviorState.WAITING_CONFIRM)
         {
            PrintTools.info("Motion START!");

            SetBooleanParameterPacket confirmPacket = new SetBooleanParameterPacket("", true);

            System.out.println("confirmPacket Dispatch");
            drcBehaviorTestHelper.getBehaviorCommunicationBridge().sendPacketToBehavior(confirmPacket);

            int numberOfDiplayedWayPoints = 10;
            WholeBodyTrajectoryToolboxOutputConverter converter = new WholeBodyTrajectoryToolboxOutputConverter(getRobotModel());
            WholeBodyTrajectoryToolboxOutputStatus wholeBodyTrajectoryToolboxOutputStatus = wayPointsByVRUIBehaviorStateMachine.getPlanConstrainedWholeBodyTrajectoryBehavior()
                                                                                                                                                 .getWholeBodyTrajectoryToolboxOutputStatus();

            for (int i = 0; i < numberOfDiplayedWayPoints; i++)
            {
               int length = wholeBodyTrajectoryToolboxOutputStatus.getTrajectoryTimes().length;
               double trajectoryTime = wholeBodyTrajectoryToolboxOutputStatus.getTrajectoryTimes()[length - 1];
               double time = trajectoryTime * (double) (i) / (double) (numberOfDiplayedWayPoints);

               KinematicsToolboxOutputStatus robotConfiguration = converter.getRobotConfiguration(wholeBodyTrajectoryToolboxOutputStatus, time);
               KinematicsToolboxOutputConverter kinematicConverter = new KinematicsToolboxOutputConverter(getRobotModel());
               kinematicConverter.updateFullRobotModel(robotConfiguration);

               FullHumanoidRobotModel robotModel = kinematicConverter.getFullRobotModel();
               Pose3D lHandPose = new Pose3D(robotModel.getHand(RobotSide.LEFT).getBodyFixedFrame().getTransformToWorldFrame());
               lHandPose.appendTranslation(0.0, -WholeBodyTrajectoryToolboxController.handCoordinateOffsetX, 0.0);
               scs.addStaticLinkGraphics(getXYZAxis(lHandPose, 0.075));

               Pose3D rHandPose = new Pose3D(robotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame());
               rHandPose.appendTranslation(0.0, WholeBodyTrajectoryToolboxController.handCoordinateOffsetX, 0.0);
               scs.addStaticLinkGraphics(getXYZAxis(rHandPose, 0.15));

               PrintTools.info("" + i + " " + rHandPose);
            }

            break;
         }
      }

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(50.0);

      System.out.println("End");

   }
}
