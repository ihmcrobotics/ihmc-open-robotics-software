package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

public class RunEvent
{
   private int runID;
   private String eventName;
   private float eventTimeInSeconds;
   private boolean isSuccessful;
   private int eventSequence;

   public RunEvent(int runID, String eventName, float eventTimeInSeconds, boolean isSuccessful)
   {
       this(runID, eventName,eventTimeInSeconds, isSuccessful, -1);
   }

   public RunEvent(int runID, String eventName, float eventTimeInSeconds, boolean isSuccessful, int eventSequence)
   {
      this.runID = runID;
      this.eventName = eventName;
      this.eventTimeInSeconds = eventTimeInSeconds;
      this.isSuccessful = isSuccessful;
      this.eventSequence = eventSequence;
   }

   public int getRunID()
   {
      return runID;
   }

   public String getEventName()
   {
      return eventName;
   }

   public float getEventTimeInSeconds()
   {
      return eventTimeInSeconds;
   }

   public boolean isSuccessful()
   {
      return isSuccessful;
   }

   public int getEventSequence()
   {
      return eventSequence;
   }
}
