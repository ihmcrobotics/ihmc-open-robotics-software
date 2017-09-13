package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

@RosMessagePacket(documentation = "This message notifies the user of a change in the high level state. This message's primary\n"
                                  + "use is to signal a requested state change is completed.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/high_level_state_change")
public class HighLevelStateChangeStatusMessage extends StatusPacket<HighLevelStateChangeStatusMessage>
{
   @RosExportedField(documentation = "initialState gives the controller's state prior to transition")
   public HighLevelController initialState;
   @RosExportedField(documentation = "endState gives the state the controller has transitioned into")
   public HighLevelController endState;
   
   public HighLevelStateChangeStatusMessage()
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
   }
   
   public HighLevelStateChangeStatusMessage(HighLevelController initialState, HighLevelController endState)
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
      setStateChange(initialState, endState);
   }
   
   public HighLevelStateChangeStatusMessage(Random random)
   {
      this.initialState = HighLevelController.values[random.nextInt(HighLevelController.values.length)];
      this.endState = HighLevelController.values[random.nextInt(HighLevelController.values.length)];
   }

   public void setStateChange(HighLevelController initialState, HighLevelController endState)
   {
      this.initialState = initialState;
      this.endState = endState;
   }

   @Override
   public void set(HighLevelStateChangeStatusMessage other)
   {
      destination = other.destination;
      initialState = other.initialState;
      endState = other.endState;
   }

   public HighLevelController getInitialState()
   {
      return initialState;
   }
   
   public HighLevelController getEndState()
   {
      return endState;
   }
   
   @Override
   public boolean epsilonEquals(HighLevelStateChangeStatusMessage other, double epsilon)
   {
      return this.getInitialState().equals(other.getInitialState()) && this.getEndState().equals(other.getEndState());
   }

}
