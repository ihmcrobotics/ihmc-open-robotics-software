package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.log.LogTools;
import us.ihmc.manipulation.planning.exploringSpatial.TrajectoryLibraryForDRC;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Disabled
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
   @Test
   public void testOneBigCircle() throws Exception, UnreasonableAccelerationException
   {
      super.testOneBigCircle();
   }

   @Override
   @Test
   public void testHandCirclePositionAndYaw() throws Exception, UnreasonableAccelerationException
   {
      super.testHandCirclePositionAndYaw();
   }

   @Override
   @Test
   public void testHandCirclePositionAndYawPitchRoll() throws Exception, UnreasonableAccelerationException
   {
      super.testHandCirclePositionAndYawPitchRoll();
   }

   @Test
   public void testDoorMotion() throws Exception, UnreasonableAccelerationException
   {
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
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(500);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeOpeningDoorTrajectory(time, trajectoryTime, openingRadius, openingAngle,
                                                                                                     openingDirectionCW, knobPose, twistTime, twistRadius,
                                                                                                     twistAngle, twistDirectionCW);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);
      Pose3D controlFramePose = new Pose3D();

      trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

      handTrajectories.add(trajectory);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // run test
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   @Test
   public void testDrillMotion() throws Exception, UnreasonableAccelerationException
   {
      // trajectory parameter
      double trajectoryTime = 10.0;

      boolean cuttingDirectionCW = true;
      double cuttingRadius = 0.3;
      Vector3D wallNormalVector = new Vector3D(-1.0, 0.0, 0.0);
      Point3D cuttingCenterPosition = new Point3D(0.7, -0.3, 1.1);

      // wbt toolbox configuration message
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> trajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

      Vector3D translationToGraspingFrame = new Vector3D(-0.0, 0.05, -0.1);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeCuttingWallTrajectory(time, trajectoryTime, cuttingRadius, cuttingDirectionCW,
                                                                                                     cuttingCenterPosition, wallNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectoryHand = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                     handFunction, selectionMatrix);
      Pose3D controlFramePose = new Pose3D();
      controlFramePose.appendTranslation(translationToGraspingFrame);

      trajectoryHand.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      trajectoryHand.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

      trajectories.add(trajectoryHand);

      ConfigurationSpaceName[] spaces = {ConfigurationSpaceName.YAW};

      rigidBodyConfigurations.add(HumanoidMessageTools.createRigidBodyExplorationConfigurationMessage(hand, spaces));

      // keep sight on trajectory.
      RigidBodyBasics head = fullRobotModel.getHead();
      SelectionMatrix6D selectionMatrixHead = new SelectionMatrix6D();
      selectionMatrixHead.clearSelection();
      selectionMatrixHead.selectLinearY(true);
      selectionMatrixHead.selectLinearZ(true);

      WaypointBasedTrajectoryMessage trajectoryHead = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(head, 0.0, trajectoryTime, timeResolution,
                                                                                                                     handFunction, selectionMatrixHead);

      trajectoryHead.getControlFramePositionInEndEffector().set(new Point3D(0.5, 0.0, 0.0));
      trajectoryHead.setWeight(0.01);
      trajectories.add(trajectoryHead);

      // run test      
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxMessage message = HumanoidMessageTools.createWholeBodyTrajectoryToolboxMessage(configuration, trajectories, null, rigidBodyConfigurations);
      runTrajectoryTest(message, maxNumberOfIterations);
   }

   @Test
   public void testValveMotion() throws Exception, UnreasonableAccelerationException
   {
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
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(1000);

      // trajectory message
      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();

      double timeResolution = trajectoryTime / 100.0;

      RobotSide robotSide = RobotSide.RIGHT;
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

      FunctionTrajectory handFunction = time -> TrajectoryLibraryForDRC.computeClosingValveTrajectory(time, trajectoryTime, closingRadius, closingDirectionCW,
                                                                                                      closingAngle, valveCenterPosition, valveNormalVector);

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      WaypointBasedTrajectoryMessage trajectory = WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution,
                                                                                                                 handFunction, selectionMatrix);
      Pose3D controlFramePose = new Pose3D();

      trajectory.getControlFramePositionInEndEffector().set(controlFramePose.getPosition());
      trajectory.getControlFrameOrientationInEndEffector().set(controlFramePose.getOrientation());

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

   @Test
   public void testReaching() throws Exception, UnreasonableAccelerationException
   {
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();

      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.getInitialConfiguration().set(HumanoidMessageTools.createKinematicsToolboxOutputStatus(fullRobotModel));
      configuration.setMaximumExpansionSize(2300);

      RigidBodyBasics hand = fullRobotModel.getHand(RobotSide.RIGHT);
      List<ReachingManifoldMessage> reachingManifolds = new ArrayList<>();

      ReachingManifoldMessage reachingManifold = HumanoidMessageTools.createReachingManifoldMessage(hand);

      reachingManifold.getManifoldOriginPosition().set(new Point3D(0.7, -0.2, 1.0));
      reachingManifold.getManifoldOriginOrientation().set(new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI * 0.5, -Math.PI * 0.5, -0.1};
      double[] upperLimits = new double[] {Math.PI * 0.5, Math.PI * 0.5, 0.0};
      HumanoidMessageTools.packManifold(ConfigurationSpaceName.toBytes(manifoldSpaces), lowerLimits, upperLimits, reachingManifold);
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

      LogTools.info("END");
   }
}
