package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.ConstrainedWholeBodyPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PlanConstrainedWholeBodyTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DrawingTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.WayPointsTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   private ConstrainedWholeBodyPlanningToolboxModule cwbPlanningToolboxModule;

   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

   private void setupCWBPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      cwbPlanningToolboxModule = new ConstrainedWholeBodyPlanningToolboxModule(robotModel, fullRobotModel, null, visulaizerOn);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
   }

   public ArrayList<Graphics3DObject> getXYZAxis(Pose3D pose, double length)
   {
      double axisHeight = length;
      double axisRadius = length/10.0;
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
      if (visualize)
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

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
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

   @Test
   public void testForWayPointsTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      RobotSide robotSide = RobotSide.LEFT;
      
      Pose3D[] poseOfWayPoints = new Pose3D[3];
      Quaternion wayPointOrientation;
      
      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendPitchRotation(Math.PI * 0.2);
      poseOfWayPoints[0] = new Pose3D(new Point3D(0.6, -0.4, 1.0), wayPointOrientation);
      
      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendYawRotation(Math.PI * 0.2);
      poseOfWayPoints[1] = new Pose3D(new Point3D(0.6,  0.0, 1.0), wayPointOrientation);
      
      wayPointOrientation = new Quaternion();
      wayPointOrientation.appendRollRotation(Math.PI * 0.2);
      poseOfWayPoints[2] = new Pose3D(new Point3D(0.6,  0.0, 1.5), wayPointOrientation);
      
      WayPointsTrajectory endeffectorTrajectory = new WayPointsTrajectory(robotSide, poseOfWayPoints, 10.0);
      
      
      scs.addStaticLinkGraphics(getXYZAxis(poseOfWayPoints[0], 0.05));
      scs.addStaticLinkGraphics(getXYZAxis(poseOfWayPoints[1], 0.05));
      scs.addStaticLinkGraphics(getXYZAxis(poseOfWayPoints[2], 0.05));
      
      
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(1.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(4.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(6.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(8.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(9.0, RobotSide.LEFT, new ConfigurationSpace()), 0.1));
      
      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(0.0, RobotSide.RIGHT, new ConfigurationSpace()), 0.2));
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }
   
   // @Test
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
       * reaching initial configuration
       */
      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.3);
      HandTrajectoryMessage lhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 1.0, new Point3D(0.0, 0.4, 0.75), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(lhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

      initialOrientation = new Quaternion();
      initialOrientation.appendPitchRotation(Math.PI * 0.3);
      HandTrajectoryMessage rhandTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 1.0, new Point3D(-0.0, -0.4, 0.75), initialOrientation,
                                                                               referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(rhandTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      sdfFullRobotModel.updateFrames();
      /*
       * Behavior create.
       */
      PlanConstrainedWholeBodyTrajectoryBehavior planningBehavior = new PlanConstrainedWholeBodyTrajectoryBehavior("PlanningBehavior", getRobotModel(),
                                                                                                                   drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                                                   sdfFullRobotModel,
                                                                                                                   drcBehaviorTestHelper.getYoTime());

      ConstrainedEndEffectorTrajectory endeffectorTrajectory = new DrawingTrajectory(20.0);
      PrintTools.info("" + endeffectorTrajectory.getTrajectoryTime());

      planningBehavior.setInputs(endeffectorTrajectory, sdfFullRobotModel);
      PlanConstrainedWholeBodyTrajectoryBehavior.constrainedEndEffectorTrajectory = endeffectorTrajectory;

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      System.out.println("Behavior Dispatch");
      drcBehaviorTestHelper.dispatchBehavior(planningBehavior);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15.0);
      System.out.println("Go Motion " + drcBehaviorTestHelper.getYoTime());

      PrintTools.info("planningResult " + planningBehavior.getConstrainedWholeBodyPlanningToolboxOutputStatus().getPlanningResult());

      ArrayList<Pose3D> handTrajectories = planningBehavior.getHandTrajectories(RobotSide.LEFT);
      int numberOfPath = handTrajectories.size();
      for (int i = 0; i < numberOfPath - 2; i++)
      {
         Pose3D pose = handTrajectories.get(i);
         scs.addStaticLinkGraphics(getXYZAxis(pose, 0.05));
      }

      System.out.println("Send " + drcBehaviorTestHelper.getYoTime());
      drcBehaviorTestHelper.send(planningBehavior.getWholebodyTrajectoryMessage());

      System.out.println("Send " + drcBehaviorTestHelper.getYoTime());
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(planningBehavior.getWholebodyTrajectoryMessage().getHandTrajectoryMessage(RobotSide.LEFT)
                                                                               .getTrajectoryTime());

      System.out.println("End " + drcBehaviorTestHelper.getYoTime());
   }
}
