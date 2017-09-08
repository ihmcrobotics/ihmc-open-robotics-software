package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

import java.util.Random;

@RosMessagePacket(documentation = "This message notifies the user of a change in the high level state. This message's primary\n"
                                  + "use is to signal a requested state change is completed.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/high_level_state_change")
public class NewHighLevelControllerStateChangeStatusMessage extends StatusPacket<NewHighLevelControllerStateChangeStatusMessage>
{
   @RosExportedField(documentation = "initialState gives the controller's state prior to transition")
   public NewHighLevelControllerStates initialState;
   @RosExportedField(documentation = "endState gives the state the controller has transitioned into")
   public NewHighLevelControllerStates endState;

   public NewHighLevelControllerStateChangeStatusMessage()
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
   }

   public NewHighLevelControllerStateChangeStatusMessage(NewHighLevelControllerStates initialState, NewHighLevelControllerStates endState)
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
      setStateChange(initialState, endState);
   }

   public NewHighLevelControllerStateChangeStatusMessage(Random random)
   {
      this.initialState = NewHighLevelControllerStates.values[random.nextInt(NewHighLevelControllerStates.values.length)];
      this.endState = NewHighLevelControllerStates.values[random.nextInt(NewHighLevelControllerStates.values.length)];
   }

   public void setStateChange(NewHighLevelControllerStates initialState, NewHighLevelControllerStates endState)
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

   public NewHighLevelControllerStates getInitialState()
   {
      return initialState;
   }
   
   public NewHighLevelControllerStates getEndState()
   {
      return endState;
   }
   
   @Override
   public boolean epsilonEquals(NewHighLevelControllerStateChangeStatusMessage other, double epsilon)
   {
      return this.getInitialState().equals(other.getInitialState()) && this.getEndState().equals(other.getEndState());
   }

}
