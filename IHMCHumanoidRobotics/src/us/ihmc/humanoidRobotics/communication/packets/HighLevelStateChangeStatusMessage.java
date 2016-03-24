package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

@ClassDocumentation("This message notifies the user of a change in the high level state. This message's primary\n"
                                  + "use is to signal a requested state change is completed.")
public class HighLevelStateChangeStatusMessage extends StatusPacket<HighLevelStateChangeStatusMessage>
{
   @FieldDocumentation("initialState gives the controller's state prior to transition")
   public HighLevelState initialState;
   @FieldDocumentation("endState gives the state the controller has transitioned into")
   public HighLevelState endState;
   
   public HighLevelStateChangeStatusMessage()
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
   }
   
   public HighLevelStateChangeStatusMessage(HighLevelState initialState, HighLevelState endState)
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
      setStateChange(initialState, endState);
   }
   
   public HighLevelStateChangeStatusMessage(Random random)
   {
      this.initialState = HighLevelState.values[random.nextInt(HighLevelState.values.length)];
      this.endState = HighLevelState.values[random.nextInt(HighLevelState.values.length)];
   }

   public void setStateChange(HighLevelState initialState, HighLevelState endState)
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

   public HighLevelState getInitialState()
   {
      return initialState;
   }
   
   public HighLevelState getEndState()
   {
      return endState;
   }
   
   @Override
   public boolean epsilonEquals(HighLevelStateChangeStatusMessage other, double epsilon)
   {
      return this.getInitialState().equals(other.getInitialState()) && this.getEndState().equals(other.getEndState());
   }

}
