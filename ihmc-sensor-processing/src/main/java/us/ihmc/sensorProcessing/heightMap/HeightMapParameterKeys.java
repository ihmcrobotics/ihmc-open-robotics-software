package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class HeightMapParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey gridResolutionXY = keys.addDoubleKey ("Grid Resolution XY", 0.05);
   public static final DoubleStoredPropertyKey gridSizeXY = keys.addDoubleKey ("Grid Size XY", 3.0);
   public static final DoubleStoredPropertyKey maxZ = keys.addDoubleKey("Max Z", 1.5);
   public static final DoubleStoredPropertyKey nominalVariance = keys.addDoubleKey("Nominal Variance", 1e-4);
   public static final IntegerStoredPropertyKey maxPointsPerCell = keys.addIntegerKey("Max Points Per Cell", 6);
   public static final DoubleStoredPropertyKey mahalonobisScale = keys.addDoubleKey ("Mahalonobis Scale", 1.5);
}
