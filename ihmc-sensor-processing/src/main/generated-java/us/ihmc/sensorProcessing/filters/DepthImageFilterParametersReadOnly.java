package us.ihmc.sensorProcessing.filters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.sensorProcessing.filters.DepthImageFilterParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilterParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getCosAngleThreshold()
   {
      return get(cosAngleThreshold);
   }

   default double getDepthThreshold()
   {
      return get(depthThreshold);
   }

   default double getNormalViewThreshold()
   {
      return get(normalViewThreshold);
   }

   default double getDepthVarianceThreshold()
   {
      return get(depthVarianceThreshold);
   }
}
