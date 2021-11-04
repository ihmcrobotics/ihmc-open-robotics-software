package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class HeightMapParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey gridResolutionXY = keys.addDoubleKey ("Grid Resolution XY", 0.03);
   public static final DoubleStoredPropertyKey gridSizeXY = keys.addDoubleKey ("Grid Size XY", 5.0);
   public static final DoubleStoredPropertyKey maxZ = keys.addDoubleKey("Max Z", 0.6);
   public static final DoubleStoredPropertyKey nominalStandardDeviation = keys.addDoubleKey("Nominal Std Dev", 0.017992);
   public static final IntegerStoredPropertyKey maxPointsPerCell = keys.addIntegerKey("Max Points Per Cell", 20);
   public static final DoubleStoredPropertyKey mahalonobisScale = keys.addDoubleKey ("Mahalonobis Scale", 2.0);
}
