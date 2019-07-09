package us.ihmc.humanoidBehaviors;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
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
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ActivationReference;

public class StepInPlaceBehavior
{
   private final BehaviorHelper behaviorHelper;

   private final ActivationReference<Boolean> stepping;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);

   public StepInPlaceBehavior(Messager messager, Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      LogTools.debug("Initializing step in place behavior");

      behaviorHelper = new BehaviorHelper(messager, robotModel, ros2Node);

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
      private static final Category Root = apiFactory.createRootCategory("StepInPlaceBehavior");
      private static final CategoryTheme StepInPlace = apiFactory.createCategoryTheme("StepInPlace");

      public static final Topic<Boolean> Stepping = Root.child(StepInPlace).topic(apiFactory.createTypedTopicTheme("Stepping"));
      public static final Topic<Boolean> Abort = Root.child(StepInPlace).topic(apiFactory.createTypedTopicTheme("Abort"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
