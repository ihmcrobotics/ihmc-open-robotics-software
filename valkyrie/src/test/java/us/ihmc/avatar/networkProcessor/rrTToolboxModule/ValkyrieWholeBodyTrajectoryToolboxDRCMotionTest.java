package us.ihmc.avatar.networkProcessor.rrTToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxSettings;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TrajectoryLibraryForDRC;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class ValkyrieWholeBodyTrajectoryToolboxDRCMotionTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
{
   private DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return getRobotModel().getSimpleRobotName();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDoorMotion() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();

      // trajectory parameter
      double trajectoryTime = 5.0;

      double openingAngle = 30.0 / 180.0 * Math.PI;
      double openingRadius = 0.8;
      boolean openingDirectionCW = true; // in X-Y plane.

      Point3D knobPosition = new Point3D(0.6, -0.25, 1.0);
      Quaternion knobOrientation = new Quaternion();
      knobOrientation.appendYawRotation(-0.02 * Math.PI);
      knobOrientation.appendRollRotation(-0.5 * Math.PI);
      Pose3D knobPose = new Pose3D(knobPosition, knobOrientation); // grasping pose

      double twistTime = 1.0;
      double twistRadius = 0.15;
      double twistAngle = 60.0 / 180.0 * Math.PI;
      boolean twistDirectionCW = true; // plane which is parallel with knobDirection

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(500);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeOpeningDoorTrajectory(time, trajectoryTime, openingRadius, openingAngle,
                                                                                                     openingDirectionCW, knobPose, twistTime, twistRadius,
                                                                                                     twistAngle, twistDirectionCW);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);

      trajectory.setControlFramePose(handControlFrames.get(robotSide));

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(new RigidBodyExplorationConfigurationMessage(hand, spaces));

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testDrillMotion() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();

      // trajectory parameter
      double trajectoryTime = 10.0;

      boolean cuttingDirectionCW = true;
      double cuttingRadius = 0.3;
      Vector3D wallNormalVector = new Vector3D(-1.0, 0.0, 0.0);
      Point3D cuttingCenterPosition = new Point3D(0.6, -0.3, 1.1);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      Vector3D translationToGraspingFrame = new Vector3D(-0.1, 0.05, -0.1);
      handControlFrames.get(robotSide).appendTranslation(translationToGraspingFrame);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeCuttingWallTrajectory(time, trajectoryTime, cuttingRadius, cuttingDirectionCW,
                                                                                                     cuttingCenterPosition, wallNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);

      trajectory.setControlFramePose(handControlFrames.get(robotSide));

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(new RigidBodyExplorationConfigurationMessage(hand, spaces));

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testValveMotion() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();

      // trajectory parameter
      double trajectoryTime = 5.0;
      boolean closingDirectionCW = false;
      double closingRadius = 0.2;
      double closingAngle = Math.PI;
      Vector3D valveNormalVector = new Vector3D(-1.0, 0.2, 0.0);
      Point3D valveCenterPosition = new Point3D(0.5, -0.4, 1.1);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeClosingValveTrajectory(time, trajectoryTime, closingRadius, closingDirectionCW,
                                                                                                      closingAngle, valveCenterPosition, valveNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction, selectionMatrix);

      trajectory.setControlFramePose(handControlFrames.get(robotSide));

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(new RigidBodyExplorationConfigurationMessage(hand, spaces));
      
      //rigidBodyConfigurations.add(new RigidBodyExplorationConfigurationMessage(fullRobotModel.getHand(RobotSide.LEFT)));

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }
}
