package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.tools.DocumentedEnum;

public enum ExecutionMode implements DocumentedEnum<ExecutionMode>
{
   OVERRIDE, QUEUE;

   @Override
   public String getDocumentation(ExecutionMode var)
   {
      switch (var)
      {
      case OVERRIDE:
         return "This message will override the previous.";
      case QUEUE:
         return "The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.";
      default:
         return "No documentation available";
      }
   }

   @Override
   public ExecutionMode[] getDocumentedValues()
   {
      return values();
   }
}
