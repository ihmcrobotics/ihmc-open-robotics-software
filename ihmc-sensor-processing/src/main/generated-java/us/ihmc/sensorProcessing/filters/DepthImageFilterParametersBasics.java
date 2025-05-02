package us.ihmc.sensorProcessing.filters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilterParametersBasics extends DepthImageFilterParametersReadOnly, StoredPropertySetBasics
{
   default void setCosAngleThreshold(double cosAngleThreshold)
   {
      set(DepthImageFilterParameters.cosAngleThreshold, cosAngleThreshold);
   }

   default void setDepthThreshold(double depthThreshold)
   {
      set(DepthImageFilterParameters.depthThreshold, depthThreshold);
   }

   default void setNormalViewThreshold(double normalViewThreshold)
   {
      set(DepthImageFilterParameters.normalViewThreshold, normalViewThreshold);
   }

   default void setDepthVarianceThreshold(double depthVarianceThreshold)
   {
      set(DepthImageFilterParameters.depthVarianceThreshold, depthVarianceThreshold);
   }
}
