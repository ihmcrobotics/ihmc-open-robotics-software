package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.WalkOverTerrainGoalPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DoorTimingBehavior.DoorTimingBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.RunEvent;
import us.ihmc.ros2.Ros2Node;

public class TimingBehaviorHelper extends AbstractBehavior
{
   public final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerResult = new AtomicReference<>(null);
   public final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(null);
   public final AtomicReference<DoorLocationPacket> doorLocationMessage = new AtomicReference<>(null);
   public final AtomicReference<WalkOverTerrainGoalPacket> walkOverTerrainGoalMessage = new AtomicReference<>(null);
   public final AtomicReference<HandTrajectoryMessage> handTrajectoryMessage = new AtomicReference<>(null);
   public final AtomicReference<ArmTrajectoryMessage> armTrajectoryMessage = new AtomicReference<>(null);

   public final AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);
   public final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>(null);
   public final AtomicReference<FootstepPlanningRequestPacket> footstepPlanningRequestPacket = new AtomicReference<>(null);
   public SQLBehaviorDatabaseManager dataBase;

   public TimingBehaviorHelper(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);

      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, plannerResult::set);
      createSubscriber(FootstepPlanningRequestPacket.class, footstepPlanningToolboxSubGenerator, footstepPlanningRequestPacket::set);

      createSubscriber(FootstepStatusMessage.class, controllerSubGenerator, footstepStatusMessage::set);
      createSubscriber(HandTrajectoryMessage.class, controllerSubGenerator, handTrajectoryMessage::set);
      createSubscriber(ArmTrajectoryMessage.class, controllerSubGenerator, armTrajectoryMessage::set);
      createSubscriber(FootstepDataListMessage.class, controllerSubGenerator, footstepDataListMessage::set);
      createSubscriber(WalkingStatusMessage.class, controllerSubGenerator, walkingStatusMessage::set);

      createSubscriber(FootstepStatusMessage.class, controllerPubGenerator, footstepStatusMessage::set);
      createSubscriber(HandTrajectoryMessage.class, controllerPubGenerator, handTrajectoryMessage::set);
      createSubscriber(ArmTrajectoryMessage.class, controllerPubGenerator, armTrajectoryMessage::set);
      createSubscriber(FootstepDataListMessage.class, controllerPubGenerator, footstepDataListMessage::set);
      createSubscriber(WalkingStatusMessage.class, controllerPubGenerator, walkingStatusMessage::set);
      createBehaviorInputSubscriber(DoorLocationPacket.class, doorLocationMessage::set);
      createBehaviorInputSubscriber(WalkOverTerrainGoalPacket.class, walkOverTerrainGoalMessage::set);
      dataBase = new SQLBehaviorDatabaseManager();
   }

   public void clean()
   {
      plannerResult.set(null);
      footstepStatusMessage.set(null);
      doorLocationMessage.set(null);
      walkOverTerrainGoalMessage.set(null);
      handTrajectoryMessage.set(null);
      armTrajectoryMessage.set(null);
      footstepDataListMessage.set(null);
      walkingStatusMessage.set(null);
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

   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone(double timeinState)
   {
      return false;
   }

   public void saveEvent(int runID, String eventName, double eventTime)
   {
      dataBase.saveRunEvent(dataBase.new RunEvent(runID, eventName, (float) eventTime, true));
   }

}
