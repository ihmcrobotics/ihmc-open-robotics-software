package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.WalkOverTerrainGoalPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;

public class BasicTimingBehavior extends AbstractBehavior
{
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerResult = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(null);
   private final AtomicReference<DoorLocationPacket> doorLocationMessage = new AtomicReference<>(null);
   private final AtomicReference<WalkOverTerrainGoalPacket> walkOverTerrainGoalMessage = new AtomicReference<>(null);
   private final AtomicReference<HandTrajectoryMessage> handTrajectoryMessage = new AtomicReference<>(null);
   private final AtomicReference<ArmTrajectoryMessage> armTrajectoryMessage = new AtomicReference<>(null);

   private final AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>(null);

   public BasicTimingBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);

      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, plannerResult::set);
      createSubscriber(FootstepStatusMessage.class  , controllerSubGenerator, footstepStatusMessage::set);
      createSubscriber(HandTrajectoryMessage.class  , controllerSubGenerator, handTrajectoryMessage::set);
      createSubscriber(ArmTrajectoryMessage.class   , controllerSubGenerator, armTrajectoryMessage::set);
      createSubscriber(FootstepDataListMessage.class, controllerSubGenerator, footstepDataListMessage::set);
      createSubscriber(WalkingStatusMessage.class   , controllerSubGenerator, walkingStatusMessage::set);
      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationMessage::set);
      createBehaviorInputSubscriber(WalkOverTerrainGoalPacket.class, walkOverTerrainGoalMessage::set);
      
      publishTextToSpeech("created timer behavior");

   }

   @Override
   public void doControl()
   {

      if (plannerResult.get() != null)
      {
         publishTextToSpeech("timer received plannerResultMessage");
         plannerResult.set(null);
      }
      if (footstepStatusMessage.get() != null)
      {
         publishTextToSpeech("timer received footstepStatusMessage");
         footstepStatusMessage.set(null);
      }
      if (handTrajectoryMessage.get() != null)
      {
         publishTextToSpeech("timer received handTrajectoryMessage");
         handTrajectoryMessage.set(null);
      }
      if (armTrajectoryMessage.get() != null)
      {
         publishTextToSpeech("timer received armTrajectoryMessage");
         armTrajectoryMessage.set(null);
      }
      if (footstepDataListMessage.get() != null)
      {
         publishTextToSpeech("timer received footstepDataListMessage");
         footstepDataListMessage.set(null);
      }
      if (walkingStatusMessage.get() != null)
      {
         publishTextToSpeech("timer received walkingStatusMessage");
         walkingStatusMessage.set(null);
      }
      if (doorLocationMessage.get() != null)
      {
         publishTextToSpeech("timer received doorLocationMessage");
         doorLocationMessage.set(null);
      }
      if (walkOverTerrainGoalMessage.get() != null)
      {
         publishTextToSpeech("timer received walkOverTerrainGoalMessage");
         walkOverTerrainGoalMessage.set(null);
      }

   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("entering timer behavior");

   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("leaving timer behavior");

   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

}
