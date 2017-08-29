package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelStates;

import java.util.Random;

@RosMessagePacket(documentation = "This message notifies the user of a change in the high level state. This message's primary\n"
                                  + "use is to signal a requested state change is completed.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/high_level_state_change")
public class NewHighLevelControllerStateChangeStatusMessage extends StatusPacket<NewHighLevelControllerStateChangeStatusMessage>
{
   @RosExportedField(documentation = "initialState gives the controller's state prior to transition")
   public NewHighLevelStates initialState;
   @RosExportedField(documentation = "endState gives the state the controller has transitioned into")
   public NewHighLevelStates endState;

   public NewHighLevelControllerStateChangeStatusMessage()
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
   }

   public NewHighLevelControllerStateChangeStatusMessage(NewHighLevelStates initialState, NewHighLevelStates endState)
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
      setStateChange(initialState, endState);
   }

   public NewHighLevelControllerStateChangeStatusMessage(Random random)
   {
      this.initialState = NewHighLevelStates.values[random.nextInt(NewHighLevelStates.values.length)];
      this.endState = NewHighLevelStates.values[random.nextInt(NewHighLevelStates.values.length)];
   }

   public void setStateChange(NewHighLevelStates initialState, NewHighLevelStates endState)
   {
      this.initialState = initialState;
      this.endState = endState;
   }

   @Override
   public void set(NewHighLevelControllerStateChangeStatusMessage other)
   {
      destination = other.destination;
      initialState = other.initialState;
      endState = other.endState;
   }

   public NewHighLevelStates getInitialState()
   {
      return initialState;
   }
   
   public NewHighLevelStates getEndState()
   {
      return endState;
   }
   
   @Override
   public boolean epsilonEquals(NewHighLevelControllerStateChangeStatusMessage other, double epsilon)
   {
      return this.getInitialState().equals(other.getInitialState()) && this.getEndState().equals(other.getEndState());
   }

}
