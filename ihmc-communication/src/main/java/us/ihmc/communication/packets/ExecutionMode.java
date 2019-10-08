package us.ihmc.communication.packets;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum ExecutionMode
{
   @RosEnumValueDocumentation(documentation = "This message will override the previous.")
   OVERRIDE,
   @RosEnumValueDocumentation(documentation = "The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.")
   QUEUE,
   @RosEnumValueDocumentation(documentation = "This message is part of a stream of messages sent to the controller at high frequency.")
   STREAM;

   public static final ExecutionMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ExecutionMode fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
