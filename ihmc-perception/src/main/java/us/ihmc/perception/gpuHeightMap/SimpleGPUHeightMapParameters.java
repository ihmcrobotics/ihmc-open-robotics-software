package us.ihmc.perception.gpuHeightMap;

public class SimpleGPUHeightMapParameters
{
   // resolution in m
   public double resolution = 0.4;
   // map's size in m
   public double mapLength = 8.0;

   // points outside this distance are outliers
   double mahalanobisThreshold = 2.0;
   // cells with higher traversability are used for drift compensation
   double traversabilityInlier = 0.9;
   // points higher than this value from the sensor will be filtered out
   double maxHeightRange = 1.0;
   // points with shorter distance will be filtered out
   double minValidDistance = 0.0; //0.2;
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   double rampedHeightRangeB = 1.0;
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   double rampedHeightRangeA = 0.3;
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   double rampedHeightRangeC = 0.2;
   // point's noise is sensor_noise_factor * z^2 (z is distance from sensor).
   double sensorNoiseFactor = 0.05;
   // if point is an outlier, and this value to the cell's variance
   double outlierVariance = 0.01;
   // initial variance for each cell.
   double initialVariance = 1000.0;
   // max variance for each cell
   double maxVariance = 100.0;
}
