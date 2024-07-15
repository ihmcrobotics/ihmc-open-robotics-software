package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryPointMessage;
import imgui.ImGui;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

//TODO move parameters of poses in Nadia
public class RDXHumanoidDemoPoses extends RDXPanel
{
   private static final double maxFootSpeed = 0.7;
   private static final double nominalFootWidth = 0.225;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXTeleoperationParameters teleoperationParameters;

   private SideDependentList<double[]> armsConfiguration = new SideDependentList<>();
   private final YawPitchRoll chestOrientation;
   private final YawPitchRoll pelvisOrientation;
   private final Point3D pelvisPosition;
   private boolean usedFirstMode = false;

   private final SideDependentList<FootLoadBearingMessage> footLoadBearingMessagesToPublish = new SideDependentList<>();
   private final AtomicReference<ChestTrajectoryMessage> chestTrajectoryMessagesToPublish = new AtomicReference<>();
   private final ArrayList<ArmTrajectoryMessage> armTrajectoryMessagesToPublish = new ArrayList<>();
   private final ArrayList<PelvisTrajectoryMessage> pelvisTrajectoryMessagesToPublish = new ArrayList<>();
   private final SideDependentList<FootTrajectoryMessage> footTrajectoryMessagesToPublish = new SideDependentList<>();

   private static final double[] rightArmGreeting1 = {0.27, -1.00, -0.682, -2.24, -0.592, -0.61, -0.79};
   private static final double[] rightArmGreeting2 = {0.253, -0.872, -1.22, -2.035, 0.073, -0.61, -0.839};

   private static final double[] leftArmSquat = new double[] {-0.7, 0.8, 0.53, -1.9};
   private static final double[] rightArmSquat = new double[] {-0.7, -0.8, -0.53, -1.9};

   private static final double[] leftArmFlex1 = new double[] {0.71, 1.4, 1.05, -2.00};
   private static final double[] rightArmFlex1 = new double[] {0.71, -1.4, -1.05, -2.00};
   private static final double[] leftArmFlex2 = new double[] {0.02, 1.1, -0.96, -2.00};
   private static final double[] rightArmFlex2 = new double[] {0.02, -1.1, 0.96, -2.00};

   private static final double[] leftArmBallet1 = new double[] {-0.39, 2.21, 0.48, -2.00};
   private static final double[] rightArmBallet1 = new double[] {0.02, -0.7, 0.44, -1.7};
   private static final double[] leftArmBallet2 = new double[] {0.6, 0.8, -0.91, -1.84};
   private static final double[] rightArmBallet2 = new double[] {-0.56, -0.31, -0.53, -2.24};

   private static final double[] leftArmKarateKid1 = new double[] {1.0, 1.22, 1.05, -1.57, -1.57, -0.61};
   private static final double[] rightArmKarateKid1 = new double[] {1.0, -1.22, -1.05, -1.57, -1.57, -0.61};

   private static final double[] leftArmKarateKid2 = new double[] {1.0, 0.0, 1.05, 0.0, 0.0, 0.0};
   private static final double[] rightArmKarateKid2 = new double[] {1.0, 0.0, -1.05, 0.0, 0.0, 0.0};

   private static final double[] leftArmRunningMan = new double[] { 1.22, 0.0, 0.1, -0.8, 0.0};
   private static final double[] rightArmRunningMan = new double[] {-0.56, -0.39, -1.12, -1.74, -0.29, 0.4, 0.45};

   private static final double[] extendedDab = new double[] {0.4, Math.toRadians(60), 0.0, 0.0};
   private static final double[] bentDab = new double[] {-0.42, -1.22, 0.33, -1.85, -1.73, -0.11, -0.156};

   private static final double homeHeight = 1.05;

   public RDXHumanoidDemoPoses(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2ControllerHelper ros2ControllerHelper,
                               RDXTeleoperationParameters teleoperationParameters)
   {
      super("Demo Poses");

      setRenderMethod(this::renderImGuiWidgets);
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;

      armsConfiguration.put(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
      armsConfiguration.put(RobotSide.RIGHT, robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
      chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
      pelvisOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
      pelvisPosition = new Point3D(0.0, 0.0, homeHeight);
   }

   private static double[] quickCopy(double[] incoming)
   {
      double[] outgoing = new double[incoming.length];
      System.arraycopy(incoming, 0, outgoing, 0, incoming.length);
      return outgoing;
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Demo Poses: ");
      ImGui.sameLine();
      // zero our poses
      pelvisOrientation.setToZero();
      pelvisPosition.setToZero();
      pelvisPosition.setZ(homeHeight);
      chestOrientation.setToZero();

      double[] leftArm = quickCopy(robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
      double[] rightArm = quickCopy(robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
      armsConfiguration.put(RobotSide.LEFT, leftArm);
      armsConfiguration.put(RobotSide.RIGHT, rightArm);

      chestTrajectoryMessagesToPublish.set(null);
      armTrajectoryMessagesToPublish.clear();
      pelvisTrajectoryMessagesToPublish.clear();
      footTrajectoryMessagesToPublish.clear();

      if (ImGui.button(labels.get("Home Pose")))
      {
         double delay = 0.0;
         for (RobotSide robotSide : RobotSide.values)
         {
            if (isFootOffGround(robotSide))
               delay = Math.max(delay, setFootDown(robotSide, 2.0));
         }

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime(), 0.1);

         appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime(), 0.1);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime(), 0.1);

         publishPoses();
         usedFirstMode = false;
      }
      if (ImGui.button(labels.get("Greeting")))
      {
         if (usedFirstMode)
         {
            System.arraycopy(rightArmGreeting1, 0, rightArm, 0, Math.min(rightArm.length, rightArmGreeting1.length));
         }
         else
         {
            System.arraycopy(rightArmGreeting2, 0, rightArm, 0, Math.min(rightArm.length, rightArmGreeting2.length));
         }

         armsConfiguration.put(RobotSide.RIGHT, rightArm);
         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), 1.0);
         appendChestOrientationToPublish(chestOrientation, 1.0);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, 1.0);

         publishPoses();

         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Squat")))
      {
         double delay = setBothFeetDown(1.0);
         double moveDuration = 3.0;

         if (!usedFirstMode)
         {
//            System.arraycopy(leftArmSquat, 0, leftArm, 0, Math.min(leftArm.length, leftArmSquat.length));
//            System.arraycopy(rightArmSquat, 0, rightArm, 0, Math.min(rightArm.length, rightArmSquat.length));
            armsConfiguration.put(RobotSide.LEFT, leftArm);
            armsConfiguration.put(RobotSide.RIGHT, rightArm);

//            pelvisOrientation.setToPitchOrientation(Math.toRadians(10.0));
            pelvisPosition.setZ(0.8);
         }
         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), moveDuration, delay);
         appendChestOrientationToPublish(chestOrientation, moveDuration, delay);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, moveDuration, delay);

         publishPoses();
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Flex")))
      {
         double delay = setBothFeetDown(teleoperationParameters.getTrajectoryTime());
         double moveDuration = 2.0;

         if (usedFirstMode)
         {
            System.arraycopy(leftArmFlex1, 0, leftArm, 0, Math.min(leftArm.length, leftArmFlex1.length));
            System.arraycopy(rightArmFlex1, 0, rightArm, 0, Math.min(rightArm.length, rightArmFlex1.length));
         }
         else
         {
            System.arraycopy(leftArmFlex2, 0, leftArm, 0, Math.min(leftArm.length, leftArmFlex2.length));
            System.arraycopy(rightArmFlex2, 0, rightArm, 0, Math.min(rightArm.length, rightArmFlex2.length));

            pelvisOrientation.setToPitchOrientation(Math.toRadians(0));
            pelvisPosition.setZ(1.0);
         }
         armsConfiguration.put(RobotSide.LEFT, leftArm);
         armsConfiguration.put(RobotSide.RIGHT, rightArm);

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), moveDuration, delay);
         appendChestOrientationToPublish(chestOrientation, moveDuration, delay);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, moveDuration, delay);
         publishPoses();
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Ballet")))
      {
         if (usedFirstMode)
         {
            System.arraycopy(leftArmBallet1, 0, leftArm, 0, Math.min(leftArm.length, leftArmBallet1.length));
            System.arraycopy(rightArmBallet1, 0, rightArm, 0, Math.min(rightArm.length, rightArmBallet1.length));

            armsConfiguration.put(RobotSide.LEFT, leftArm);
            armsConfiguration.put(RobotSide.RIGHT, rightArm);

            chestOrientation.setToYawOrientation(Math.toRadians(25));
            pelvisOrientation.setYaw(Math.toRadians(15));
            pelvisOrientation.setPitch(Math.toRadians(0));

            FramePose3D rightLiftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
            rightLiftFootPose.getPosition().addZ(0.03);
            rightLiftFootPose.changeFrame(ReferenceFrame.getWorldFrame());

            FramePose3D rightFinalFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleZUpFrame(RobotSide.LEFT));
            rightFinalFootPose.setZ(0.2);
            rightFinalFootPose.setY(-0.5);
            rightFinalFootPose.appendYawRotation(Math.toRadians(30.0));
            rightFinalFootPose.appendPitchRotation(Math.toRadians(45.0));
            rightFinalFootPose.appendRollRotation(Math.toRadians(-25.0));

            double[] durations = new double[] {0.1, teleoperationParameters.getTrajectoryTime()};
            FramePose3D[] poses = new FramePose3D[] {rightLiftFootPose, rightFinalFootPose};

            createFootPoseMessage(RobotSide.RIGHT, durations, poses);

            for (RobotSide robotSide : RobotSide.values)
               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime(), 0.1);
            appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime(), 0.1);
            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime(), 0.1);
         }
         else
         {
            System.arraycopy(leftArmBallet2, 0, leftArm, 0, Math.min(leftArm.length, leftArmBallet2.length));
            System.arraycopy(rightArmBallet2, 0, rightArm, 0, Math.min(rightArm.length, rightArmBallet2.length));

            armsConfiguration.put(RobotSide.LEFT, leftArm);
            armsConfiguration.put(RobotSide.RIGHT, rightArm);

            chestOrientation.setToZero();
            pelvisOrientation.setToPitchOrientation(Math.toRadians(10));
            pelvisPosition.setZ(0.90);

            setBothFeetDown(teleoperationParameters.getTrajectoryTime() + 0.25);

            for (RobotSide robotSide : RobotSide.values)
               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime(), 0.25);
            appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime(), 0.25);
            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime(), 0.25);
         }

         publishPoses();
         usedFirstMode = !usedFirstMode;
      }

//      if (ImGui.button(labels.get("Karate Kid")))
//      {
//         if (!usedFirstMode)
//         {
//            System.arraycopy(leftArmKarateKid1, 0, leftArm, 0, Math.min(leftArm.length, leftArmKarateKid1.length));
//            System.arraycopy(rightArmKarateKid1, 0, rightArm, 0, Math.min(rightArm.length, rightArmKarateKid1.length));
//
//            armsConfiguration.put(RobotSide.LEFT, leftArm);
//            armsConfiguration.put(RobotSide.RIGHT, rightArm);
//
//            double duration = 1.0;
//
//            RobotSide kickSide;
//            if (isFootOffGround(RobotSide.RIGHT))
//            {
//               kickSide = RobotSide.RIGHT;
//            }
//            else
//            {
//               kickSide = RobotSide.LEFT;
//            }
//            FramePose3D footPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(kickSide.getOppositeSide()));
//            footPose.getPosition().addZ(0.45);
//            footPose.getPosition().addX(0.25);
//            footPose.getPosition().setY(kickSide.negateIfRightSide(0.15));
//            createFootPoseMessage(kickSide, new double[] {duration}, new FramePose3D[] {footPose});
//
//            for (RobotSide robotSide : RobotSide.values)
//               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), duration);
//            appendChestOrientationToPublish(chestOrientation, duration);
//            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, duration);
//         }
//         else
//         {
//            System.arraycopy(leftArmKarateKid2, 0, leftArm, 0, Math.min(leftArm.length, leftArmKarateKid2.length));
//            System.arraycopy(rightArmKarateKid2, 0, rightArm, 0, Math.min(rightArm.length, rightArmKarateKid2.length));
//
//            double duration = 1.0;
//
//            RobotSide kickSide;
//            if (isFootOffGround(RobotSide.RIGHT))
//            {
//               kickSide = RobotSide.RIGHT;
//            }
//            else
//            {
//               kickSide = RobotSide.LEFT;
//            }
//            FramePose3D footPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(kickSide.getOppositeSide()));
//            footPose.getPosition().addZ(0.5);
//            footPose.getPosition().addX(0.7);
//            footPose.getPosition().addY(kickSide.negateIfRightSide(0.15));
//            footPose.getOrientation().setToPitchOrientation(Math.toRadians(-45.0));
//            createFootPoseMessage(kickSide, new double[] {duration}, new FramePose3D[] {footPose});
//
//            armsConfiguration.put(RobotSide.LEFT, leftArm);
//            armsConfiguration.put(RobotSide.RIGHT, rightArm);
//
//            for (RobotSide robotSide : RobotSide.values)
//               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), duration);
//            appendChestOrientationToPublish(chestOrientation, duration);
//            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, duration);
//         }
//
//         publishPoses();
//         usedFirstMode = !usedFirstMode;
//      }

      if (ImGui.button(labels.get("Running Man")))
      {
         System.arraycopy(leftArmRunningMan, 0, leftArm, 0, Math.min(leftArm.length, leftArmRunningMan.length));
         System.arraycopy(rightArmRunningMan, 0, rightArm, 0, Math.min(rightArm.length, rightArmRunningMan.length));

         armsConfiguration.put(RobotSide.LEFT, leftArm);
         armsConfiguration.put(RobotSide.RIGHT, rightArm);

         double duration = 2.5;

         RobotSide kickSide;
         if (isFootOffGround(RobotSide.RIGHT))
         {
            kickSide = RobotSide.RIGHT;
         }
         else
         {
            kickSide = RobotSide.LEFT;
         }
         FramePose3D footPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(kickSide.getOppositeSide()));
         footPose.getPosition().setZ(0.6);
         footPose.getPosition().setX(-0.35);
         footPose.getPosition().setY(kickSide.negateIfRightSide(0.15));
         footPose.changeFrame(ReferenceFrame.getWorldFrame());
         footPose.getOrientation().setYawPitchRoll(0.0, 0.8 * Math.PI / 2.0, 0.0);
         createFootPoseMessage(kickSide, new double[] {duration}, new FramePose3D[] {footPose});

         pelvisOrientation.setToPitchOrientation(Math.toRadians(10));

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), duration);
         appendChestOrientationToPublish(chestOrientation, duration, 0.05);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, duration, 0.05);

         publishPoses();
      }

      if (ImGui.button(labels.get("High Kick")))
      {
         armsConfiguration.put(RobotSide.LEFT, leftArm);
         armsConfiguration.put(RobotSide.RIGHT, rightArm);

         double duration = 2.0;

         RobotSide kickSide;
         if (isFootOffGround(RobotSide.RIGHT))
         {
            kickSide = RobotSide.RIGHT;
         }
         else
         {
            kickSide = RobotSide.LEFT;
         }
         FramePose3D footPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(kickSide.getOppositeSide()));
         footPose.getPosition().setZ(0.5);
         footPose.getPosition().setX(0.7);
         footPose.getPosition().setY(kickSide.negateIfRightSide(0.175));
         footPose.getOrientation().setYawPitchRoll(0.0, -Math.toRadians(70.0), 0.0);
         createFootPoseMessage(kickSide, new double[] {duration}, new FramePose3D[] {footPose});

         pelvisOrientation.setToPitchOrientation(-Math.toRadians(30.0));

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), duration);
         appendChestOrientationToPublish(chestOrientation, duration, 0.05);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, duration, 0.05);

         publishPoses();
      }

      if (ImGui.button(labels.get("Dab")))
      {
         double duration = 4.0;

         RobotSide kickSide;
         if (isFootOffGround(RobotSide.RIGHT))
         {
            kickSide = RobotSide.RIGHT;
            System.arraycopy(bentDab, 0, leftArm, 0, Math.min(leftArm.length, bentDab.length));
            System.arraycopy(extendedDab, 0, rightArm, 0, Math.min(rightArm.length, extendedDab.length));
            leftArm[1] = -leftArm[1];
            leftArm[2] = -leftArm[2];
            if (leftArm.length > 4)
            {
               leftArm[4] = -leftArm[4];
               leftArm[5] = -leftArm[5];
            }
            rightArm[1] = -rightArm[1];
            rightArm[2] = -rightArm[2];
            if (rightArm.length > 4)
            {
               rightArm[4] = -rightArm[4];
               rightArm[5] = -rightArm[5];
            }
         }
         else
         {
            System.arraycopy(bentDab, 0, rightArm, 0, Math.min(rightArm.length, bentDab.length));
            System.arraycopy(extendedDab, 0, leftArm, 0, Math.min(leftArm.length, extendedDab.length));
            kickSide = RobotSide.LEFT;
         }


         FramePose3D footPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(kickSide.getOppositeSide()));
         footPose.getPosition().setZ(0.35);
         footPose.getPosition().setX(0.0);
         footPose.getPosition().setY(kickSide.negateIfRightSide(0.75));
         footPose.getOrientation().setToRollOrientation(kickSide.negateIfRightSide(Math.toRadians(60)));

         pelvisOrientation.setToRollOrientation(Math.toRadians(kickSide.negateIfRightSide(15.0)));

         createFootPoseMessage(kickSide, new double[]{duration}, new FramePose3D[]{footPose});

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), duration);
         appendChestOrientationToPublish(chestOrientation, duration, 0.05);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, duration, 0.05);

         publishPoses();
      }
   }

   private boolean isFootOffGround(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return syncedRobot.getLatestCapturabilityBasedStatus().getLeftFootSupportPolygon3d().isEmpty();
      else
         return syncedRobot.getLatestCapturabilityBasedStatus().getRightFootSupportPolygon3d().isEmpty();
   }

   private double setBothFeetDown(double totalDuration)
   {
      boolean movedFoot = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         if (isFootOffGround(robotSide))
         {
            movedFoot = true;
            totalDuration = setFootDown(robotSide, totalDuration);
         }
      }

      if (movedFoot)
         return totalDuration;

      return 0.0;
   }

   private double setFootDown(RobotSide robotSide, double totalDuration)
   {
      FramePose3D finalFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide()));
      finalFootPose.setY(robotSide.negateIfRightSide(nominalFootWidth)); // TODO extract this parameter

      FramePose3D intermediatePose = new FramePose3D(finalFootPose);
      intermediatePose.getPosition().setZ(0.075);
      finalFootPose.getPosition().setZ(-0.02);

      intermediatePose.changeFrame(ReferenceFrame.getWorldFrame());
      finalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D currentFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
      currentFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      double distanceToIntermediate = currentFootPose.getPositionDistance(intermediatePose);
      double distanceToFinal = intermediatePose.getPositionDistance(finalFootPose);

      double intermediateAlpha = 0.7;
      double durationScale = (distanceToIntermediate / (intermediateAlpha * totalDuration)) / maxFootSpeed;
      durationScale = Math.max((distanceToFinal / ((1.0 - intermediateAlpha) * totalDuration)) / maxFootSpeed, durationScale);
      durationScale = Math.max(durationScale, 1.0);

      totalDuration = durationScale * totalDuration;

      double[] durations = new double[] {intermediateAlpha * totalDuration,(1.0 - intermediateAlpha) * totalDuration};
      FramePose3D[] waypoints = new FramePose3D[]{intermediatePose, finalFootPose};
      createFootPoseMessage(robotSide, durations, waypoints);

      FootLoadBearingMessage loadBearingMessage = HumanoidMessageTools.createFootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD);
      loadBearingMessage.setExecutionDelayTime(totalDuration + 0.1);

      footLoadBearingMessagesToPublish.put(robotSide, loadBearingMessage);

      return totalDuration;
   }

   private void createFootPoseMessage(RobotSide robotSide, double[] moveDurations, FramePose3DReadOnly[] footPoses)
   {
      FramePose3D lastFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
      lastFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage();
      footTrajectoryMessage.setRobotSide(robotSide.toByte());
      footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().clear();
      footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());

      double waypointTime = 0.0;

      for (int i = 0; i < moveDurations.length; i++)
      {
         FramePose3D poseToPublish = new FramePose3D(footPoses[i]);
         poseToPublish.changeFrame(ReferenceFrame.getWorldFrame());

         double minDuration = poseToPublish.getPositionDistance(lastFootPose) / maxFootSpeed;
         double moveDuration = Math.max(moveDurations[i], minDuration);

         waypointTime += moveDuration;

         SE3TrajectoryPointMessage trajectoryPoint = footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.getPosition().set(poseToPublish.getPosition());
         trajectoryPoint.getOrientation().set(poseToPublish.getOrientation());
         trajectoryPoint.setTime(waypointTime);

         lastFootPose.set(poseToPublish);
      }

      footTrajectoryMessagesToPublish.put(robotSide, footTrajectoryMessage);
   }

   private void appendPelvisOrientationToPublish(Orientation3DReadOnly pelvisOrientation, Point3DReadOnly pelvisPosition, double trajectoryDuration)
   {
      appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, trajectoryDuration, 0.0);
   }

   private void appendPelvisOrientationToPublish(Orientation3DReadOnly pelvisOrientation,
                                                 Point3DReadOnly pelvisPosition,
                                                 double trajectoryDuration,
                                                 double executionDelay)
   {
      FramePose3D pelvisPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      pelvisPose.changeFrame(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame());
      pelvisPose.getTranslation().setZ(pelvisPosition.getZ());
      pelvisPose.getRotation().set(pelvisOrientation);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisMessage = new PelvisTrajectoryMessage();
      pelvisMessage.getSe3Trajectory()
                   .set(HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryDuration,
                                                                        pelvisPose.getPosition(),
                                                                        pelvisPose.getOrientation(),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      pelvisMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);

      pelvisMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);

      pelvisTrajectoryMessagesToPublish.add(pelvisMessage);
   }

   private void appendChestOrientationToPublish(Orientation3DReadOnly chestOrientation, double trajectoryDuration)
   {
      appendChestOrientationToPublish(chestOrientation, trajectoryDuration, 0.0);
   }

   private void appendChestOrientationToPublish(Orientation3DReadOnly chestOrientation, double trajectoryDuration, double executionDelay)
   {
      ChestTrajectoryMessage chestTrajectoryMessage = chestTrajectoryMessagesToPublish.get();
      if (chestTrajectoryMessage == null)
      {
         chestTrajectoryMessage = new ChestTrajectoryMessage();
         chestTrajectoryMessagesToPublish.set(chestTrajectoryMessage);
      }
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(syncedRobot.getReferenceFrames().getPelvisFrame(), chestOrientation);

      if (chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
      {
         chestTrajectoryMessage.getSo3Trajectory()
                               .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryDuration,
                                                                                    desiredChestOrientation,
                                                                                    EuclidCoreTools.zeroVector3D,
                                                                                    desiredChestOrientation.getReferenceFrame()));
      }
      else
      {
         SO3TrajectoryPointMessage point = chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().getLast();
         SO3TrajectoryPointMessage newPoint = chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         newPoint.getOrientation().set(desiredChestOrientation);
         newPoint.getAngularVelocity().setToZero();
         newPoint.setTime(point.getTime() + trajectoryDuration);
      }

      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setZSelected(true);

      chestTrajectoryMessage.getSo3Trajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);
   }

   private void appendArmTrajectoryMessageToPublish(RobotSide side, double[] armConfiguration, double trajectoryDuration)
   {
      appendArmTrajectoryMessageToPublish(side, armConfiguration, trajectoryDuration, 0.0);
   }

   private void appendArmTrajectoryMessageToPublish(RobotSide side, double[] armConfiguration, double trajectoryDuration, double executionDelay)
   {
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                  trajectoryDuration,
                                                                                                  armConfiguration);
      armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);

      armTrajectoryMessagesToPublish.add(armTrajectoryMessage);
   }

   public void publishPoses()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footTrajectoryMessagesToPublish.get(robotSide) != null)
         {
            ros2ControllerHelper.publishToController(footTrajectoryMessagesToPublish.get(robotSide));
            footTrajectoryMessagesToPublish.put(robotSide, null);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootLoadBearingMessage footLoadBearingMessage = footLoadBearingMessagesToPublish.get(robotSide);
         if (footLoadBearingMessage != null)
            ros2ControllerHelper.publishToController(footLoadBearingMessage);
         footLoadBearingMessagesToPublish.put(robotSide, null);
      }

      while (!armTrajectoryMessagesToPublish.isEmpty())
         ros2ControllerHelper.publishToController(armTrajectoryMessagesToPublish.remove(0));

      if (chestTrajectoryMessagesToPublish.get() != null)
         ros2ControllerHelper.publishToController(chestTrajectoryMessagesToPublish.getAndSet(null));

      while (!pelvisTrajectoryMessagesToPublish.isEmpty())
         ros2ControllerHelper.publishToController(pelvisTrajectoryMessagesToPublish.remove(0));

      footTrajectoryMessagesToPublish.clear();
      footLoadBearingMessagesToPublish.clear();
      armTrajectoryMessagesToPublish.clear();
      chestTrajectoryMessagesToPublish.set(null);
      pelvisTrajectoryMessagesToPublish.clear();
   }
}
