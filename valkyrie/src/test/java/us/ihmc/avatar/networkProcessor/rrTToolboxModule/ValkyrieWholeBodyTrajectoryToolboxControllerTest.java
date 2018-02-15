package us.ihmc.avatar.networkProcessor.rrTToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
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
public class ValkyrieWholeBodyTrajectoryToolboxControllerTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testOneBigCircle() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      super.testOneBigCircle();
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testHandCirclePositionAndYaw() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      super.testHandCirclePositionAndYaw();
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testHandCirclePositionAndYawPitchRoll() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      super.testHandCirclePositionAndYawPitchRoll();
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
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      trajectory.setControlFramePose(handControlFrames.get(robotSide));

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
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
      Point3D cuttingCenterPosition = new Point3D(0.7, -0.3, 1.1);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> trajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBody hand = fullRobotModel.getHand(robotSide);

      Vector3D translationToGraspingFrame = new Vector3D(-0.0, 0.05, -0.1);
      handControlFrames.get(robotSide).appendTranslation(translationToGraspingFrame);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeCuttingWallTrajectory(time, trajectoryTime, cuttingRadius, cuttingDirectionCW,
                                                                                                     cuttingCenterPosition, wallNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectoryHand = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                     handFunction, selectionMatrix);

      trajectoryHand.setControlFramePose(handControlFrames.get(robotSide));

      trajectories.add(trajectoryHand);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // keep sight on trajectory.
      RigidBody head = fullRobotModel.getHead();
      SelectionMatrix6D selectionMatrixHead = new SelectionMatrix6D();
      selectionMatrixHead.clearSelection();
      selectionMatrixHead.selectLinearY(true);
      selectionMatrixHead.selectLinearZ(true);

      WaypointBasedTrajectoryMessage trajectoryHead = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(head, 0.0, trajectoryTime, timeResolution,
                                                                                                                     handFunction, selectionMatrixHead);

      trajectoryHead.setControlFramePosition(new Point3D(0.5, 0.0, 0.0));
      trajectoryHead.setWeight(0.01);
      trajectories.add(trajectoryHead);

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, trajectories, null, rigidBodyConfigurations);
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
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);

      trajectory.setControlFramePose(handControlFrames.get(robotSide));

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // to hold left hand.
      //rigidBodyConfigurations.add(new RigidBodyExplorationConfigurationMessage(fullRobotModel.getHand(RobotSide.LEFT)));

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testReaching() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(2300);

      RigidBody hand = fullRobotModel.getHand(RobotSide.RIGHT);
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      ReachingManifoldMessage reachingManifold = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifold.setOrigin(new Point3D(0.7, -0.2, 1.0), new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI * 0.5, -Math.PI * 0.5, -0.1};
      double[] upperLimits = new double[] {Math.PI * 0.5, Math.PI * 0.5, 0.0};
      reachingManifold.setManifold(manifoldSpaces, lowerLimits, upperLimits);
      reachingManifolds.add(reachingManifold);

      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      /**
       * BIT   @link https://arxiv.org/pdf/1405.5848.pdf
       * RABIT @link https://www.ri.cmu.edu/pub_files/2016/5/main.pdf.
       * Implement BIT, RABIT for adaptive random regions.
       */
      // test for position only.
      ConfigurationSpaceName[] explorationSpaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,};
      double[] explorationUpperLimits = {0.15, 0.05, 0.2};
      double[] explorationLowerLimits = {-0.0, -0.5, -0.2};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, explorationSpaces, explorationUpperLimits, explorationLowerLimits));

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, null, reachingManifolds, rigidBodyConfigurations);

      // run toolbox
      runReachingTest(message, maxNumberOfIterations);

      PrintTools.info("END");
   }
}
