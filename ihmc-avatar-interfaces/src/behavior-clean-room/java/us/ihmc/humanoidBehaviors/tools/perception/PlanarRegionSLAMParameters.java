package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class PlanarRegionSLAMParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList(PlanarRegionSLAMParameters.class);

   public static final IntegerStoredPropertyKey iterations = keys.addIntegerKey("Iterations");
   public static final DoubleStoredPropertyKey boundingBoxHeight = keys.addDoubleKey("Bounding box height");
   public static final DoubleStoredPropertyKey minimumNormalDotProduct = keys.addDoubleKey("Minimum normal dot product");  //Math.cos(Math.toRadians(5.0));
   public static final DoubleStoredPropertyKey dampedLeastSquaresLambda = keys.addDoubleKey("Damped least squares lambda");

   public PlanarRegionSLAMParameters()
   {
      super(keys,
            PlanarRegionSLAMParameters.class,
            "ihmc-open-robotics-software",
            "ihmc-avatar-interfaces/src/behavior-clean-room/resources");

      load();
   }

   public double getBoundingBoxHeight()
   {
      return get(boundingBoxHeight);
   }

   public void setBoundingBoxHeight(double boundingBoxHeight)
   {
      set(PlanarRegionSLAMParameters.boundingBoxHeight, boundingBoxHeight);
   }

   public int getIterations()
   {
      return get(iterations);
   }

   public void setIterations(int iterations)
   {
      set(PlanarRegionSLAMParameters.iterations, iterations);
   }

   public double getMinimumNormalDotProduct()
   {
      return get(minimumNormalDotProduct);
   }

   public void setMinimumNormalDotProduct(double minimumNormalDotProduct)
   {
      set(PlanarRegionSLAMParameters.minimumNormalDotProduct, minimumNormalDotProduct);
   }

   public double getDampedLeastSquaresLambda()
   {
      return get(dampedLeastSquaresLambda);
   }

   public void setDampedLeastSquaresLambda(double dampedLeastSquaresLambda)
   {
      set(PlanarRegionSLAMParameters.dampedLeastSquaresLambda, dampedLeastSquaresLambda);
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();
      planarRegionSLAMParameters.save();
   }
}
