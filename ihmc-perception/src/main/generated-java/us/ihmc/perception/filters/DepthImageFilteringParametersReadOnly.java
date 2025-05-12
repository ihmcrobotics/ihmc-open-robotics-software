package us.ihmc.perception.filters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.filters.DepthImageFilteringParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilteringParametersReadOnly extends StoredPropertySetReadOnly
{
   default int getLongLengthForRectangle()
   {
      return get(longLengthForRectangle);
   }

   default int getShortLengthForRectangle()
   {
      return get(shortLengthForRectangle);
   }

   default double getCosineThreshold()
   {
      return get(cosineThreshold);
   }

   default double getNormalThreshold()
   {
      return get(normalThreshold);
   }

   default int getRANSACIterations()
   {
      return get(ransacIterations);
   }

   default int getMinimumNormalsFound()
   {
      return get(minimumNormalsFound);
   }
}
