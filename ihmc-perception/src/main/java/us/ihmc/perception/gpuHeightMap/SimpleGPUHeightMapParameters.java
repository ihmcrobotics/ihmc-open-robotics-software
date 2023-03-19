package us.ihmc.perception.gpuHeightMap;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class SimpleGPUHeightMapParameters extends StoredPropertySet implements SimpleGPUHeightMapParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   // resolution in m
   public static final DoubleStoredPropertyKey resolution = keys.addDoubleKey("Resolution");
   // map's size in m
   public static final DoubleStoredPropertyKey mapLength = keys.addDoubleKey("Map length");

   // points higher than this value from the sensor will be filtered out
   public static final DoubleStoredPropertyKey maxHeightRange = keys.addDoubleKey("Max height range");
   // points with shorter distance will be filtered out
   public static final DoubleStoredPropertyKey minValidDistance = keys.addDoubleKey("Min valid distance");
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   public static final DoubleStoredPropertyKey rampedHeightRangeB = keys.addDoubleKey("Ramped height range B");
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   public static final DoubleStoredPropertyKey rampedHeightRangeA = keys.addDoubleKey("Ramped height range A");
   // if z > max(d - rapmed_height_range_b, 0) * ramped_height_range_a + ramped_height_range_c, reject
   public static final DoubleStoredPropertyKey rampedHeightRangeC = keys.addDoubleKey("Ramped height range C");

   public SimpleGPUHeightMapParameters()
   {
      super(keys, SimpleGPUHeightMapParameters.class);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, SimpleGPUHeightMapParameters.class);
      parameters.loadUnsafe();
      parameters.save();
   }
}
