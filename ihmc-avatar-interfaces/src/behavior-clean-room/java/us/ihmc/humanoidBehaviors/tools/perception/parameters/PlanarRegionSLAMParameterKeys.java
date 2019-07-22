package us.ihmc.humanoidBehaviors.tools.perception.parameters;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class PlanarRegionSLAMParameterKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList("planarRegionSLAMParameters");

   public static final IntegerStoredPropertyKey iterations = keys.addIntegerKey("Iterations");
   public static final DoubleStoredPropertyKey boundingBoxHeight = keys.addDoubleKey("Bounding box height");
   public static final DoubleStoredPropertyKey minimumNormalDotProduct = keys.addDoubleKey("Minimum normal dot product");  //Math.cos(Math.toRadians(5.0));
   public static final DoubleStoredPropertyKey dampedLeastSquaresLambda = keys.addDoubleKey("Damped least squares lambda");

   public static void main(String[] args)
   {
      StoredPropertySet.printInitialSaveFileContents(keys.keys());
   }
}
