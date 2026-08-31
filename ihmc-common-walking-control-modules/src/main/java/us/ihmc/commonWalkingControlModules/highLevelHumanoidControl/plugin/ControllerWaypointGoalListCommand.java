package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.ControllerWaypointGoalListMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

public class ControllerWaypointGoalListCommand implements Command<ControllerWaypointGoalListCommand, ControllerWaypointGoalListMessage>
{
   private long sequenceId;
   private boolean queueWaypoints;
   private final RecyclingArrayList<ControllerWaypointGoalCommand> waypoints = new RecyclingArrayList<>(ControllerWaypointGoalCommand::new);

   public ControllerWaypointGoalListCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      queueWaypoints = false;
      waypoints.clear();
   }

   @Override
   public void setFromMessage(ControllerWaypointGoalListMessage message)
   {
      sequenceId = message.getSequenceId();
      queueWaypoints = message.getQueueWaypoints();
      waypoints.clear();
      for (int i = 0; i < message.getWaypoints().size(); i++)
      {
         waypoints.add().setFromMessage(message.getWaypoints().get(i));
      }
   }

   @Override
   public Class<ControllerWaypointGoalListMessage> getMessageClass()
   {
      return ControllerWaypointGoalListMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !waypoints.isEmpty();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(ControllerWaypointGoalListCommand other)
   {
      sequenceId = other.sequenceId;
      queueWaypoints = other.queueWaypoints;
      waypoints.clear();
      for (int i = 0; i < other.waypoints.size(); i++)
      {
         waypoints.add().set(other.waypoints.get(i));
      }
   }

   public boolean getQueueWaypoints()
   {
      return queueWaypoints;
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public void addWaypoint(ControllerWaypointGoalCommand waypoint)
   {
      waypoints.add().set(waypoint);
   }

   public ControllerWaypointGoalCommand getWaypoint(int index)
   {
      return waypoints.get(index);
   }
}
