package us.ihmc.perception.filters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.filters.DepthImageFilteringParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilteringParametersReadOnly extends StoredPropertySetReadOnly
{
   default int getWindowSizeInPixels()
   {
      return get(windowSizeInPixels);
   }

   default double getAngleThresholdInRadians()
   {
      return get(angleThresholdInRadians);
   }

   default double getNormalAngleThreshold()
   {
      return get(normalAngleThreshold);
   }

   default int getRANSACIterations()
   {
      return get(ransacIterations);
   }

   default int getMinimumDepthValuesRequiredInWindow()
   {
      return get(minimumDepthValuesRequiredInWindow);
   }
}
