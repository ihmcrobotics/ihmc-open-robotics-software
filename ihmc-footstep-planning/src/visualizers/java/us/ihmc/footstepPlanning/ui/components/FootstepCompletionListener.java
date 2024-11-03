package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import javafx.animation.AnimationTimer;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepCompletionListener extends AnimationTimer
{
   private final Messager messager;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessageReference;
   private final AtomicReference<FootstepDataListMessage> activeFootstepPlan;

   public FootstepCompletionListener(Messager messager)
   {
      this.messager = messager;
      footstepStatusMessageReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepStatusMessage);
      activeFootstepPlan = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse);
   }

   @Override
   public void handle(long now)
   {
      FootstepStatusMessage footstepStatusMessage = footstepStatusMessageReference.getAndSet(null);
      if (footstepStatusMessage == null || FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) != FootstepStatus.COMPLETED)
      {
         return;
      }
      
      FootstepDataListMessage activePlan = activeFootstepPlan.get();
      if (activePlan == null || activePlan.getFootstepDataList().isEmpty())
      {
         return;
      }

      FootstepDataMessage firstStepCurrentPlan = activePlan.getFootstepDataList().get(0);
      if (firstStepCurrentPlan.getRobotSide() != footstepStatusMessage.getRobotSide())
      {
         return;
      }

      activePlan.getFootstepDataList().remove(0);
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, activePlan);
   }
}
