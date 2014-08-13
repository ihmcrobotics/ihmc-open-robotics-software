package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;


public class DoNothingVariousWalkingProviderFactory extends ComponentBasedVariousWalkingProviderFactory
{   
   public DoNothingVariousWalkingProviderFactory(double controlDT)
   {
      super(false, null, controlDT);
   }
}

