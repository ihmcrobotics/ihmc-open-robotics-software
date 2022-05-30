package us.ihmc.perception.gpuHeightMap;

public class SimpleGPUHeightMapParameters
{
   // resolution in m
   public double resolution = 0.025;
   // map's size in m
   public double mapLength = 3.5;
   
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
}
