package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This message notifies the user of a change in the high level state. This message's primary\n"
                                  + "use is to signal a requested state change is completed.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/output/high_level_state_change")
public class HighLevelStateChangeStatusMessage extends SettablePacket<HighLevelStateChangeStatusMessage>
{
   @RosExportedField(documentation = "initialState gives the controller's state prior to transition")
   public byte initialHighLevelControllerName;
   @RosExportedField(documentation = "endState gives the state the controller has transitioned into")
   public byte endHighLevelControllerName;
   
   public HighLevelStateChangeStatusMessage()
   {
      this.destination = (byte)PacketDestination.ROS_API.ordinal();
   }

   public void setStateChange(byte initialState, byte endState)
   {
      this.initialHighLevelControllerName = initialState;
      this.endHighLevelControllerName = endState;
   }

   @Override
   public void set(HighLevelStateChangeStatusMessage other)
   {
      destination = other.destination;
      initialHighLevelControllerName = other.initialHighLevelControllerName;
      endHighLevelControllerName = other.endHighLevelControllerName;
      setPacketInformation(other);
   }

   public byte getInitialState()
   {
      return initialHighLevelControllerName;
   }
   
   public byte getEndState()
   {
      return endHighLevelControllerName;
   }
   
   @Override
   public boolean epsilonEquals(HighLevelStateChangeStatusMessage other, double epsilon)
   {
      return initialHighLevelControllerName == other.initialHighLevelControllerName && endHighLevelControllerName == other.endHighLevelControllerName;
   }

}
