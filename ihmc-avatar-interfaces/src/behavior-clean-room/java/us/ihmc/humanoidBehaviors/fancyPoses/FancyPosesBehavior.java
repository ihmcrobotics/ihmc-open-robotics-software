package us.ihmc.humanoidBehaviors.fancyPoses;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.ActivationReference;

public class FancyPosesBehavior
{
   private final BehaviorHelper behaviorHelper;

   private final ActivationReference<Boolean> stepping;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);

   private final RobotSide supportSide = RobotSide.RIGHT;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final double trajectoryTime = 3.0;

   public FancyPosesBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing FancyPosesBehavior");

      this.behaviorHelper = behaviorHelper;

      behaviorHelper.createFootstepStatusCallback(this::consumeFootstepStatus);
      stepping = behaviorHelper.createBooleanActivationReference(API.Stepping, false, true);
      messager.registerTopicListener(API.Abort, this::doOnAbort);
      
      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::stepInPlace, 1, TimeUnit.SECONDS);
   }

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         behaviorHelper.shutdownScheduledThread();
      }
   }

   private void consumeFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.info("consumeFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps.");
      }
   }

   private void goToSingleSupport()
   {
//      FullHumanoidRobotModel fullRobotModel = behaviorHelper.pollFullRobotModel();
//
//      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
//      
//      ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportSide);
//      FramePose3D anklePose = new FramePose3D(ankleZUpFrame);
//      anklePose.prependTranslation(0.0, supportSide.negateIfLeftSide(0.25), 0.15);
//      anklePose.changeFrame(worldFrame);
//      Point3D position = new Point3D();
//      Quaternion orientation = new Quaternion();
//      anklePose.get(position, orientation);
//      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(supportSide.getOppositeSide(), trajectoryTime, position, orientation);
//      message.setDestination(PacketDestination.CONTROLLER.ordinal());
//      
//      networkingManager.publishToController(message);
//
//      GoHomeMessage chestGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime);
//      chestGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
//      networkingManager.publishToController(chestGoHomeMessage);
//
//      GoHomeMessage pelvisGoHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime);
//      pelvisGoHomeMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
//      networkingManager.publishToController(pelvisGoHomeMessage);
//
//      ArmTrajectoryMessage armTrajectoryMessage = getArmTrajectoryMessage(RobotSide.LEFT, trajectoryTime, HandPresets.leftHandWiderHomeJointAngles);
//      armTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
//      networkingManager.publishToController(armTrajectoryMessage);
//
//      armTrajectoryMessage = getArmTrajectoryMessage(RobotSide.RIGHT, trajectoryTime, HandPresets.rightHandWiderHomeJointAngles);
//      armTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
//      networkingManager.publishToController(armTrajectoryMessage);
   }
   
   private void stepInPlace()
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

            FullHumanoidRobotModel fullRobotModel = behaviorHelper.pollFullRobotModel();
            FootstepDataListMessage footstepList = createTwoStepInPlaceSteps(fullRobotModel);
            behaviorHelper.publishFootstepList(footstepList);
         }
      }
      else if (stepping.hasChanged())
      {
         LogTools.info("Stopped stepping");
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

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("FancyPosesBehavior");
      private static final CategoryTheme FancyPoses = apiFactory.createCategoryTheme("FancyPoses");

      public static final Topic<Boolean> Stepping = Root.child(FancyPoses).topic(apiFactory.createTypedTopicTheme("Stepping"));
      public static final Topic<Boolean> Abort = Root.child(FancyPoses).topic(apiFactory.createTypedTopicTheme("Abort"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
