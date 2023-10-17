package us.ihmc.behaviors.fancyPoses;

import java.util.concurrent.atomic.AtomicInteger;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.LocalOnlyBehaviorTreeNodeExecutor;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;

/**
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class FancyPosesBehavior extends LocalOnlyBehaviorTreeNodeExecutor implements Destroyable
{
   private final BehaviorHelper helper;

   private final ActivationReference<Boolean> stepping = null;
   private final Notification goToSingleSupportNotification = new Notification();
   private final Notification goToDoubleSupportNotification = new Notification();
   private final Notification goToRunningManNotification = new Notification();
   private final Notification goToKarateKid1Notification = new Notification();
   private final Notification goToKarateKid2Notification = new Notification();
   private final Notification goToKarateKid3Notification = new Notification();
   private final Notification goToPresentNotification = new Notification();
   private final Notification goToShutdownPoseNotification = new Notification();

   private final AtomicInteger footstepsTaken = new AtomicInteger(2);

   private final RobotSide supportSide = RobotSide.RIGHT;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double trajectoryTime = 3.0;
   private final PausablePeriodicThread mainThread;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final ROS2SyncedRobotModel syncedRobot;

   public FancyPosesBehavior(BehaviorHelper helper)
   {
      LogTools.debug("Initializing FancyPosesBehavior");

      this.helper = helper;
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();

      robotInterface.createFootstepStatusCallback(this::acceptFootstepStatus);
//      stepping = helper.subscribeViaActivationReference(API.Stepping);

//      helper.subscribeViaCallback(API.GoToSingleSupport, object -> goToSingleSupportNotification.set());
//      helper.subscribeViaCallback(API.GoToDoubleSupport, object -> goToDoubleSupportNotification.set());
//      helper.subscribeViaCallback(API.GoToRunningMan, object -> goToRunningManNotification.set());
//      helper.subscribeViaCallback(API.GoToKarateKid1, object -> goToKarateKid1Notification.set());
//      helper.subscribeViaCallback(API.GoToKarateKid2, object -> goToKarateKid2Notification.set());
//      helper.subscribeViaCallback(API.GoToKarateKid3, object -> goToKarateKid3Notification.set());
//      helper.subscribeViaCallback(API.GoToPresent, object -> goToPresentNotification.set());
//      helper.subscribeViaCallback(API.GoToShutdownPose, object -> goToShutdownPoseNotification.set());
//
//      helper.subscribeViaCallback(API.Abort, this::doOnAbort);

      mainThread = helper.createPausablePeriodicThread(getClass(), 1.0, this::doBehavior);
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   public void setEnabled(boolean enabled)
   {
      LogTools.info("Fancy poses behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
   }

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         mainThread.stop();
      }
   }

   private void acceptFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.info("acceptFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps.");
      }
   }

   private void goToSingleSupport()
   {
      syncedRobot.update();

      ReferenceFrame ankleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D anklePose = new FramePose3D(ankleZUpFrame);
      anklePose.prependTranslation(0.0, supportSide.negateIfLeftSide(0.25), 0.15);
      anklePose.changeFrame(worldFrame);
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      anklePose.get(position, orientation);

      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, anklePose);
      robotInterface.requestChestGoHome(trajectoryTime);
      robotInterface.requestPelvisGoHome(trajectoryTime);

      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, leftHandWiderHomeJointAngles);
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, rightHandWiderHomeJointAngles);

      robotInterface.requestFootLoadBearing(supportSide, LoadBearingRequest.LOAD);
      robotInterface.requestFootLoadBearing(supportSide.getOppositeSide(), LoadBearingRequest.LOAD);
   }

   private void goToDoubleSupport()
   {
      syncedRobot.update();

      ReferenceFrame ankleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D anklePose = new FramePose3D(ankleZUpFrame);
      anklePose.prependTranslation(0.0, supportSide.negateIfLeftSide(0.25), 0.0);
      anklePose.changeFrame(worldFrame);
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      anklePose.get(position, orientation);

      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, anklePose);
      robotInterface.requestChestGoHome(trajectoryTime);
      robotInterface.requestPelvisGoHome(trajectoryTime);

      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, leftHandWiderHomeJointAngles);
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, rightHandWiderHomeJointAngles);
   }

   public void goToRunningManPose()
   {
      syncedRobot.update();

      ReferenceFrame supportAnkleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D footPose = new FramePose3D(supportAnkleZUpFrame);
      footPose.getPosition().set(-0.40, supportSide.negateIfLeftSide(0.25), 0.40);
      footPose.getOrientation().setYawPitchRoll(0.0, 0.8 * Math.PI / 2.0, 0.0);
      footPose.changeFrame(worldFrame);
      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, footPose);

      FrameQuaternion chestOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, Math.toRadians(20.0), 0.0);
      chestOrientation.changeFrame(worldFrame);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      FrameQuaternion pelvisOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, Math.toRadians(10.0), 0.0);
      pelvisOrientation.changeFrame(worldFrame);
      robotInterface.requestPelvisOrientationTrajectory(trajectoryTime, pelvisOrientation);

      double[] jointAngles = new double[] {-1.2584547081002637, 0.016489729127989374, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {-0.77, 0.0, 3.0, -1.57, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   public void goToKarateKid1Pose()
   {
      syncedRobot.update();

      ReferenceFrame supportAnkleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D footPose = new FramePose3D(supportAnkleZUpFrame);
      footPose.getPosition().set(0.10, supportSide.negateIfLeftSide(0.25), 0.20);
      footPose.changeFrame(worldFrame);
      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, footPose);

      robotInterface.requestChestGoHome(trajectoryTime);
      robotInterface.requestPelvisGoHome(trajectoryTime);

      double[] jointAngles = new double[] {0.0, 0.0, 2.35, 0.76, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.0, 0.0, 2.35, -0.76, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   public void goToKarateKid2Pose()
   {
      syncedRobot.update();

      ReferenceFrame supportAnkleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D footPose = new FramePose3D(supportAnkleZUpFrame);
      footPose.getPosition().set(0.6, supportSide.negateIfLeftSide(0.25), 0.25);
      footPose.getOrientation().setYawPitchRoll(0.0, -Math.PI / 4.0, 0.0);
      footPose.changeFrame(worldFrame);
      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, footPose);

      FrameQuaternion chestOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, Math.toRadians(-5.0), 0.0);
      chestOrientation.changeFrame(worldFrame);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      FrameQuaternion pelvisOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, Math.toRadians(-15), 0.0);
      pelvisOrientation.changeFrame(worldFrame);
      robotInterface.requestPelvisOrientationTrajectory(trajectoryTime, pelvisOrientation);

      double[] jointAngles = new double[] {-0.63, -0.815, 1.78, 1.40, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.63, 0.815, 1.78, -1.40, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   public void goToKarateKid3Pose()
   {
      syncedRobot.update();

      ReferenceFrame supportAnkleZUpFrame = syncedRobot.getReferenceFrames().getAnkleZUpFrame(supportSide);
      FramePose3D footPose = new FramePose3D(supportAnkleZUpFrame);
      footPose.getPosition().set(0.00, supportSide.negateIfLeftSide(0.65), 0.2);
      footPose.getOrientation().setYawPitchRoll(0.0, 0.0, supportSide.negateIfLeftSide(Math.toRadians(40.0)));
      footPose.changeFrame(worldFrame);
      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, footPose);

      FrameQuaternion chestOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, 0.0, supportSide.negateIfLeftSide(Math.toRadians(30.0)));
      chestOrientation.changeFrame(worldFrame);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      FrameQuaternion pelvisOrientation = new FrameQuaternion(supportAnkleZUpFrame, 0.0, 0.0, supportSide.negateIfLeftSide(Math.toRadians(20.0)));
      pelvisOrientation.changeFrame(worldFrame);
      robotInterface.requestPelvisOrientationTrajectory(trajectoryTime, pelvisOrientation);

      double[] jointAngles = new double[] {-0.02, -0.017, 1.93, 0.10, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {1.28, 0.0, 1.56, -1.57, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   public void goToPresentPose()
   {
      syncedRobot.update();

      ReferenceFrame soleFrame = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT);
      double trajectoryTime = 5.0;

      FramePose3D footPose = new FramePose3D(soleFrame);
      footPose.getPosition().set(-0.511, 0.160, 0.277);
      footPose.getOrientation().set(0.008, 0.263, 0.216, 0.940);
      footPose.changeFrame(worldFrame);
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      footPose.get(position, orientation);
      robotInterface.requestFootTrajectory(supportSide.getOppositeSide(), trajectoryTime, position, orientation);

      FrameQuaternion chestOrientation = new FrameQuaternion(soleFrame, 0.039, 0.147, 0.398, 0.905);
      chestOrientation.changeFrame(worldFrame);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      FrameQuaternion pelvisOrientation = new FrameQuaternion(soleFrame, 0.014, 0.156, 0.175, 0.972);
      FramePoint3D pelvisPosition = new FramePoint3D(soleFrame, -0.158, -0.048, 0.739);
      pelvisOrientation.changeFrame(worldFrame);
      pelvisPosition.changeFrame(worldFrame);
      robotInterface.requestPelvisTrajectory(trajectoryTime, pelvisPosition, pelvisOrientation);

      double[] jointAngles = new double[] {0.765356719493866, 0.024195531383156776, 2.9822821617126465, 1.6808037757873535, -0.3247416913509369,
            0.67205411195755, 0.15090779960155487};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.10722935199737549, -0.23587453365325928, 2.419130802154541, -0.9118338823318481, -2.2621233463287354, -0.5176281929016113,
            0.005108347628265619};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   public void goToShutdownPose()
   {
      robotInterface.requestChestGoHome(trajectoryTime);
      robotInterface.requestPelvisGoHome(trajectoryTime);

      double[] jointAngles = new double[] {0.0, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      robotInterface.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }

   private void doBehavior()
   {
      if (stepping.poll())
      {
         if (stepping.hasChanged())
         {
            LogTools.info("Starting to step");
         }

         if (footstepsTaken.compareAndSet(2, 0))
         {
            LogTools.info("Sending steps");

            syncedRobot.update();
            FootstepDataListMessage footstepList = createTwoStepInPlaceSteps(syncedRobot.getFullRobotModel());
            robotInterface.requestWalk(footstepList);
         }
      }
      else if (stepping.hasChanged())
      {
         LogTools.info("Stopped stepping");
      }

      if (goToSingleSupportNotification.poll())
      {
         LogTools.info("Going to Single Support!");
         goToSingleSupport();
      }

      if (goToDoubleSupportNotification.poll())
      {
         LogTools.info("Going to Double Support!");
         goToDoubleSupport();
      }

      if (goToRunningManNotification.poll())
      {
         LogTools.info("Going to Running Man!");
         goToRunningManPose();
      }

      if (goToKarateKid1Notification.poll())
      {
         LogTools.info("Going to KarateKid1!");
         goToKarateKid1Pose();
      }

      if (goToKarateKid2Notification.poll())
      {
         LogTools.info("Going to KarateKid2!");
         goToKarateKid2Pose();
      }

      if (goToKarateKid3Notification.poll())
      {
         LogTools.info("Going to KarateKid3!");
         goToKarateKid3Pose();
      }

      if (goToPresentNotification.poll())
      {
         LogTools.info("Going to Present!");
         goToPresentPose();
      }

      if (goToShutdownPoseNotification.poll())
      {
         LogTools.info("Going to Shutdown Pose!");
         goToShutdownPose();
      }
   }

   private FootstepDataListMessage createTwoStepInPlaceSteps(FullHumanoidRobotModel fullRobotModel)
   {
      FootstepDataListMessage footstepList = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepList.getFootstepDataList();

      for (RobotSide side : RobotSide.values)
      {
         MovingReferenceFrame stepFrame = fullRobotModel.getSoleFrame(side);
         FramePoint3D footLocation = new FramePoint3D(stepFrame);
         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation);
         footstepDataMessages.add().set(footstepDataMessage);
      }
      footstepList.setAreFootstepsAdjustable(true);
      return footstepList;
   }

   public static final double[] leftHandWiderHomeJointAngles = new double[] {0.785398, -0.5237988813979918, 2.377081269248866, 2.35619, -0.33780669067363706,
         0.20773039981059624, -0.026599098661993822};
   public static final double[] rightHandWiderHomeJointAngles = new double[] {-0.785398, 0.5143374964757462, 2.2503094898479272, -2.132492022530739,
         -0.22447272781774874, -0.4780687104960028, -0.24919417978503655};

   public static class API
   {

   }
}
