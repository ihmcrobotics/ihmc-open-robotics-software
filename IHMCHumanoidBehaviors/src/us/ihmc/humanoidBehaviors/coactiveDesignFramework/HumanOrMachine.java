package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

public enum HumanOrMachine
{
   HUMAN, MACHINE;
   
   @Override
   public String toString()
   {
      switch (this)
      {
      case HUMAN:
         return "Human";
      case MACHINE:
         return "Machine";
      default:
         return null;
      }
   }
}