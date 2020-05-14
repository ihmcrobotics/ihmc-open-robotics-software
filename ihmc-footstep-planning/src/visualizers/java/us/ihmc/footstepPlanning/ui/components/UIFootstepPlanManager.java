package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
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
      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponse, this::updatePaths);

      ignorePartialFootholds = messager.createInput(FootstepPlannerMessagerAPI.IgnorePartialFootholds, false);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.IgnorePartialFootholds, this::updatePartialFootholds);

      overrideStepTimings = messager.createInput(FootstepPlannerMessagerAPI.OverrideStepTimings, false);
      manualSwingTime = messager.createInput(FootstepPlannerMessagerAPI.ManualSwingTime);
      manualTransferTime = messager.createInput(FootstepPlannerMessagerAPI.ManualTransferTime);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.OverrideStepTimings, value -> updateStepTimings());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ManualSwingTime, value -> updateStepTimings());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ManualTransferTime, value -> updateStepTimings());

      overrideStepHeight = messager.createInput(FootstepPlannerMessagerAPI.OverrideSwingHeight, false);
      manualSwingHeight = messager.createInput(FootstepPlannerMessagerAPI.ManualSwingHeight);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.OverrideSwingHeight, value -> updateStepHeights());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ManualSwingHeight, value -> updateStepHeights());

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ManualStepAdjustment, this::updateStepPlacements);

      // Send plan to robot when requested
      Runnable dispathPlanRunnable = () -> messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobot, adjustedPath.get());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.SendPlan, send -> new Thread(dispathPlanRunnable).start());
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

   private void updateStepPlacements(Pair<Integer, Pose3D> manualStepAdjustment)
   {
      FootstepDataListMessage adjustedPath = this.adjustedPath.get();
      FootstepDataListMessage plannedPath = this.plannedPath.get();

      if (manualStepAdjustment.getKey() >= adjustedPath.getFootstepDataList().size() || manualStepAdjustment.getKey() >= plannedPath.getFootstepDataList().size())
      {
         return;
      }

      int stepIndex = manualStepAdjustment.getKey();

      Point3D plannedLocation = plannedPath.getFootstepDataList().get(stepIndex).getLocation();
      Quaternion plannedOrientation = plannedPath.getFootstepDataList().get(stepIndex).getOrientation();

      adjustedPath.getFootstepDataList().get(stepIndex).getLocation().set(plannedLocation);
      adjustedPath.getFootstepDataList().get(stepIndex).getOrientation().set(plannedOrientation);

      adjustedPath.getFootstepDataList().get(stepIndex).getLocation().add(manualStepAdjustment.getValue().getPosition());
      adjustedPath.getFootstepDataList().get(stepIndex).getOrientation().append(manualStepAdjustment.getValue().getOrientation());
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

      this.plannedPath.set(plannedPath);
      adjustedPath.set(plannedPath);
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
      messageToSet.getFootstepDataList().forEach(message -> message.setSwingDuration(manualSwingTime));
      messageToSet.getFootstepDataList().forEach(message -> message.setTransferDuration(manualTransferTime));
   }

   private static void copyStepTimes(FootstepDataListMessage messageToCopyFrom, FootstepDataListMessage messageToSet)
   {
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
