package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.messager.Messager;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This class keeps track of the original planned path and any user changes to it.
 *
 * The field {@link #plannedPath} is the plan received from the planner/post-process pipeline
 * The field {@link #adjustedPath} is the plan sent to the robot
 */
public class UIFootstepPlanManager
{
   private final Messager messager;

   private final AtomicReference<FootstepDataListMessage> plannedPath = new AtomicReference<>();
   private final AtomicReference<FootstepDataListMessage> adjustedPath = new AtomicReference<>();

   // Override contact points
   private final AtomicReference<Boolean> ignorePartialFootholds;

   // Override swing and transfer times
   private final AtomicReference<Boolean> overrideStepTimings;
   private final AtomicReference<Double> manualSwingTime;
   private final AtomicReference<Double> manualTransferTime;

   // Override swing height
   private final AtomicReference<Boolean> overrideStepHeight;
   private final AtomicReference<Double> manualSwingHeight;

   public UIFootstepPlanManager(Messager messager)
   {
      this.messager = messager;
      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponse, this::updatePaths);

      ignorePartialFootholds = messager.createInput(FootstepPlannerMessagerAPI.IgnorePartialFootholds, false);
      messager.addTopicListener(FootstepPlannerMessagerAPI.IgnorePartialFootholds, this::updatePartialFootholds);

      overrideStepTimings = messager.createInput(FootstepPlannerMessagerAPI.OverrideStepTimings, false);
      manualSwingTime = messager.createInput(FootstepPlannerMessagerAPI.ManualSwingTime);
      manualTransferTime = messager.createInput(FootstepPlannerMessagerAPI.ManualTransferTime);
      messager.addTopicListener(FootstepPlannerMessagerAPI.OverrideStepTimings, value -> updateStepTimings());
      messager.addTopicListener(FootstepPlannerMessagerAPI.ManualSwingTime, value -> updateStepTimings());
      messager.addTopicListener(FootstepPlannerMessagerAPI.ManualTransferTime, value -> updateStepTimings());

      overrideStepHeight = messager.createInput(FootstepPlannerMessagerAPI.OverrideSwingHeight, false);
      manualSwingHeight = messager.createInput(FootstepPlannerMessagerAPI.ManualSwingHeight);
      messager.addTopicListener(FootstepPlannerMessagerAPI.OverrideSwingHeight, value -> updateStepHeights());
      messager.addTopicListener(FootstepPlannerMessagerAPI.ManualSwingHeight, value -> updateStepHeights());

      messager.addTopicListener(FootstepPlannerMessagerAPI.ManuallyAdjustmentedStep, this::updateStepPlacements);
      messager.addTopicListener(FootstepPlannerMessagerAPI.OverrideSpecificSwingTime, this::updateSpecificStepSwingTime);

      // Send plan to robot when requested
      Runnable dispathPlanRunnable = () -> messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobot, adjustedPath.get());
      messager.addTopicListener(FootstepPlannerMessagerAPI.SendPlan, send ->
      {
         if (adjustedPath.get() != null)
         {
            new Thread(dispathPlanRunnable, "SendPlan").start();
         }
      });
   }

   private void updatePartialFootholds(boolean ignorePartialFootholds)
   {
      if (ignorePartialFootholds)
      {
         removePartialFootholds(adjustedPath.get());
      }
      else
      {
         copyFootholds(plannedPath.get(), adjustedPath.get());
      }
   }

   private void updateStepTimings()
   {
      if (overrideStepTimings.get())
      {
         setManualStepTimes(adjustedPath.get(), manualSwingTime.get(), manualTransferTime.get());
      }
      else
      {
         copyStepTimes(plannedPath.get(), adjustedPath.get());
      }
   }

   private void updateStepHeights()
   {
      if (overrideStepHeight.get())
      {
         setManualSwingHeight(adjustedPath.get(), manualSwingHeight.get());
      }
      else
      {
         copySwingHeight(plannedPath.get(), adjustedPath.get());
      }
   }

   private void updateStepPlacements(Pair<Integer, Pose3D> manuallyAdjustedStep)
   {
      FootstepDataListMessage adjustedPath = this.adjustedPath.get();

      if (manuallyAdjustedStep.getKey() == null || manuallyAdjustedStep.getValue() == null)
      {
         return;
      }

      if (manuallyAdjustedStep.getKey() >= adjustedPath.getFootstepDataList().size())
      {
         return;
      }

      int stepIndex = manuallyAdjustedStep.getKey();
      FootstepDataMessage footstepDataMessage = adjustedPath.getFootstepDataList().get(stepIndex);
      footstepDataMessage.getLocation().set(manuallyAdjustedStep.getValue().getPosition());
      footstepDataMessage.getOrientation().set(manuallyAdjustedStep.getValue().getOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepToUpdateViz, Pair.of(stepIndex, footstepDataMessage));
   }

   private void updateSpecificStepSwingTime(Pair<Integer, Double> specificStepSwingTime)
   {
      int index = specificStepSwingTime.getKey();
      if (index >= 0 && index < adjustedPath.get().getFootstepDataList().size())
      {
         adjustedPath.get().getFootstepDataList().get(index).setSwingDuration(specificStepSwingTime.getValue());
      }
   }

   private void updatePaths(FootstepDataListMessage plannedPath)
   {
      FootstepDataListMessage adjustedPath = new FootstepDataListMessage();

      if (ignorePartialFootholds.get())
      {
         removePartialFootholds(adjustedPath);
      }

      if (overrideStepTimings.get())
      {
         setManualStepTimes(adjustedPath, manualSwingTime.get(), manualTransferTime.get());
      }

      adjustedPath.set(plannedPath);
      this.plannedPath.set(plannedPath);
      this.adjustedPath.set(adjustedPath);
   }

   private static void removePartialFootholds(FootstepDataListMessage footstepDataListMessage)
   {
      footstepDataListMessage.getFootstepDataList().forEach(message -> message.getPredictedContactPoints2d().clear());
   }

   private static void copyFootholds(FootstepDataListMessage messageToCopyFrom, FootstepDataListMessage messageToSet)
   {
      for (int i = 0; i < messageToSet.getFootstepDataList().size(); i++)
      {
         Object<Point3D> contactPointsToCopy = messageToCopyFrom.getFootstepDataList().get(i).getPredictedContactPoints2d();
         Object<Point3D> contactPointsToSet = messageToSet.getFootstepDataList().get(i).getPredictedContactPoints2d();
         contactPointsToSet.set(contactPointsToCopy);
      }
   }

   private static void setManualStepTimes(FootstepDataListMessage messageToSet, double manualSwingTime, double manualTransferTime)
   {
      if (messageToSet == null)
         return;

      messageToSet.getFootstepDataList().forEach(message -> message.setSwingDuration(manualSwingTime));
      messageToSet.getFootstepDataList().forEach(message -> message.setTransferDuration(manualTransferTime));
   }

   private static void copyStepTimes(FootstepDataListMessage messageToCopyFrom, FootstepDataListMessage messageToSet)
   {
      if (messageToCopyFrom == null || messageToSet == null)
         return;

      for (int i = 0; i < messageToSet.getFootstepDataList().size(); i++)
      {
         double swingTimeToCopy = messageToCopyFrom.getFootstepDataList().get(i).getSwingDuration();
         double transferTimeToCopy = messageToCopyFrom.getFootstepDataList().get(i).getTransferDuration();
         messageToSet.getFootstepDataList().get(i).setSwingDuration(swingTimeToCopy);
         messageToSet.getFootstepDataList().get(i).setTransferDuration(transferTimeToCopy);
      }
   }

   private static void setManualSwingHeight(FootstepDataListMessage messageToSet, double swingHeight)
   {
      messageToSet.getFootstepDataList().forEach(message -> message.setSwingHeight(swingHeight));
   }

   private static void copySwingHeight(FootstepDataListMessage messageToCopyFrom, FootstepDataListMessage messageToSet)
   {
      for (int i = 0; i < messageToSet.getFootstepDataList().size(); i++)
      {
         double swingHeightToCopy = messageToCopyFrom.getFootstepDataList().get(i).getSwingHeight();
         messageToSet.getFootstepDataList().get(i).setSwingHeight(swingHeightToCopy);
      }
   }
}
