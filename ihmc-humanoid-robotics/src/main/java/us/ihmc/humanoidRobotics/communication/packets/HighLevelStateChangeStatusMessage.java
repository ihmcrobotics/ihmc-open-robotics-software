package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This message notifies the user of a change in the high level state. This message's primary\n"
      + "use is to signal a requested state change is completed.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/output/high_level_state_change")
public class HighLevelStateChangeStatusMessage extends Packet<HighLevelStateChangeStatusMessage>
{
   public static final byte DO_NOTHING_BEHAVIOR = 0;
   public static final byte STAND_PREP_STATE = 1;
   public static final byte STAND_READY = 2;
   public static final byte FREEZE_STATE = 3;
   public static final byte STAND_TRANSITION_STATE = 4;
   public static final byte WALKING = 5;
   public static final byte DIAGNOSTICS = 6;
   public static final byte CALIBRATION = 7;

   @RosExportedField(documentation = "initialState gives the controller's state prior to transition")
   public byte initialHighLevelControllerName;
   @RosExportedField(documentation = "endState gives the state the controller has transitioned into")
   public byte endHighLevelControllerName;

   public HighLevelStateChangeStatusMessage()
   {
      this.destination = (byte) PacketDestination.ROS_API.ordinal();
   }

   @Override
   public void set(HighLevelStateChangeStatusMessage other)
   {
      destination = other.destination;
      initialHighLevelControllerName = other.initialHighLevelControllerName;
      endHighLevelControllerName = other.endHighLevelControllerName;
      setPacketInformation(other);
   }

   public void setInitialHighLevelControllerName(byte initialHighLevelControllerName)
   {
      this.initialHighLevelControllerName = initialHighLevelControllerName;
   }

   public void setEndHighLevelControllerName(byte endHighLevelControllerName)
   {
      this.endHighLevelControllerName = endHighLevelControllerName;
   }

   public byte getInitialHighLevelControllerName()
   {
      return initialHighLevelControllerName;
   }

   public byte getEndHighLevelControllerName()
   {
      return endHighLevelControllerName;
   }

   @Override
   public boolean epsilonEquals(HighLevelStateChangeStatusMessage other, double epsilon)
   {
      return initialHighLevelControllerName == other.initialHighLevelControllerName && endHighLevelControllerName == other.endHighLevelControllerName;
   }

}
