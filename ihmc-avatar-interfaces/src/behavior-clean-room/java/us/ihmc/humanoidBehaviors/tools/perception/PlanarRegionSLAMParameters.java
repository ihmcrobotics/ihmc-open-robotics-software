package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.humanoidBehaviors.tools.perception.parameters.PlanarRegionSLAMParameterKeys;
import us.ihmc.tools.property.StoredPropertySet;

public class PlanarRegionSLAMParameters extends StoredPropertySet
{
   public PlanarRegionSLAMParameters()
   {
      super(PlanarRegionSLAMParameterKeys.keys,
            PlanarRegionSLAMParameters.class,
            "ihmc-open-robotics-software",
            "ihmc-avatar-interfaces/src/behavior-clean-room/resources");
   }

   public double getBoundingBoxHeight()
   {
      return get(PlanarRegionSLAMParameterKeys.boundingBoxHeight);
   }

   public void setBoundingBoxHeight(double boundingBoxHeight)
   {
      set(PlanarRegionSLAMParameterKeys.boundingBoxHeight, boundingBoxHeight);
   }

   public int getIterations()
   {
      return get(PlanarRegionSLAMParameterKeys.iterations);
   }

   public void setIterations(int iterations)
   {
      set(PlanarRegionSLAMParameterKeys.iterations, iterations);
   }

   public double getMinimumNormalDotProduct()
   {
      return get(PlanarRegionSLAMParameterKeys.minimumNormalDotProduct);
   }

   public void setMinimumNormalDotProduct(double minimumNormalDotProduct)
   {
      set(PlanarRegionSLAMParameterKeys.minimumNormalDotProduct, minimumNormalDotProduct);
   }

   public double getDampedLeastSquaresLambda()
   {
      return get(PlanarRegionSLAMParameterKeys.dampedLeastSquaresLambda);
   }

   public void setDampedLeastSquaresLambda(double dampedLeastSquaresLambda)
   {
      set(PlanarRegionSLAMParameterKeys.dampedLeastSquaresLambda, dampedLeastSquaresLambda);
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();
      planarRegionSLAMParameters.load();
      planarRegionSLAMParameters.save();
   }
}
