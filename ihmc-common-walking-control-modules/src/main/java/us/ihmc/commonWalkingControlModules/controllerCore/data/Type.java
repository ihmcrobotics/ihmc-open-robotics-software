package us.ihmc.commonWalkingControlModules.controllerCore.data;

public enum Type
{
   DESIRED("Desired"),
   CURRENT("Current"),
   FEEDFORWARD("FeedForward"),
   FEEDBACK("Feedback"),
   ACHIEVED("Achieved"),
   ERROR("Error"),
   ERROR_CUMULATED("ErrorCumulated"),
   ERROR_INTEGRATED("ErrorIntegrated");

   private final String name;

   private Type(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }
}