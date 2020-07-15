package us.ihmc.humanoidBehaviors;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.ActivationReference;
import us.ihmc.tools.thread.PausablePeriodicThread;

public class StepInPlaceBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Step in Place", StepInPlaceBehavior::new, API.create());

   private final BehaviorHelper helper;

   private final ActivationReference<Boolean> stepping;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private final AtomicLong lastFootstepTakenID = new AtomicLong(0);
   private final AtomicLong footstepID = new AtomicLong();
   private final PausablePeriodicThread mainThread;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final RemoteSyncedRobotModel syncedRobot;

   public StepInPlaceBehavior(BehaviorHelper helper)
   {
      LogTools.debug("Initializing step in place behavior");

      this.helper = helper;
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();

      robotInterface.createFootstepStatusCallback(this::consumeFootstepStatus);
      stepping = helper.createBooleanActivationReference(API.Stepping);
      helper.createUICallback(API.Abort, this::doOnAbort);

      mainThread = helper.createPausablePeriodicThread(getClass(), 1.0, this::stepInPlace);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Step in place behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         mainThread.stop();
      }
   }

   private void consumeFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.info("consumeFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         lastFootstepTakenID.set(footstepStatusMessage.getSequenceId());
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps. Last one had id: " + lastFootstepTakenID.get());
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

         //         if (!behaviorHelper.isRobotWalking())
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
         footstepDataMessage.setSequenceId(footstepID.incrementAndGet());
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
