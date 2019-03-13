package us.ihmc.humanoidBehaviors.ui.references;

public class Activator
{
   private boolean activated = false;
   private boolean previousValue = false;
   private boolean activationChanged = false;

   public boolean poll()
   {
      activationChanged = activated != previousValue;
      previousValue = activated;
      return activated;
   }

   public boolean read()
   {
      return previousValue;
   }

   public boolean activationChanged()
   {
      return activationChanged;
   }

   public void activate()
   {
      activated = true;
   }

   public void deactivate()
   {
      activated = false;
   }
}
