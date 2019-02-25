package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

public class BipedalSupportPlanarRegionParameters
{
   private boolean enableBipedalSupportPlanarRegions = true;
   private double supportRegionScaleFactor = 2.0;

   public void setEnableBipedalSupportPlanarRegions(boolean enableBipedalSupportPlanarRegions)
   {
      this.enableBipedalSupportPlanarRegions = enableBipedalSupportPlanarRegions;
   }

   public void setSupportRegionScaleFactor(double supportRegionScaleFactor)
   {
      this.supportRegionScaleFactor = supportRegionScaleFactor;
   }

   public boolean isEnableBipedalSupportPlanarRegions()
   {
      return enableBipedalSupportPlanarRegions;
   }

   public double getSupportRegionScaleFactor()
   {
      return supportRegionScaleFactor;
   }
}
