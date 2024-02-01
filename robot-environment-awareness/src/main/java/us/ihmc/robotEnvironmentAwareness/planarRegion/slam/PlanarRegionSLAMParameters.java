package us.ihmc.robotEnvironmentAwareness.planarRegion.slam;

import us.ihmc.log.LogTools;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

/**
 * Parameters for the planar region SLAM algorithm.
 * <p>
 * On construction, these properties are loaded from the INI file:
 * ihmc-high-level-behaviors/src/main/resources/us/ihmc/humanoidBehaviors/tools/perception/planarRegionSLAMParameters.ini
 * <p>
 * To save/load properties, call the save() and load() methods.
 * <p>
 * After adding new properties, call the main() method to add them to the INI file with null values. Then,
 * set the initial value in the file.
 * <p>
 * The named getters and setters are more of a "sugar" feature for this class and may go out of date as parameters are added.
 * <p>
 * The set(Key, value) and get(Key) methods are always available.
 */
public class PlanarRegionSLAMParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final IntegerStoredPropertyKey iterationsForMatching = keys.addIntegerKey("Iterations for matching");
   public static final DoubleStoredPropertyKey boundingBoxHeight = keys.addDoubleKey("Bounding box height");
   public static final DoubleStoredPropertyKey minimumNormalDotProduct = keys.addDoubleKey("Minimum normal dot product");
   public static final DoubleStoredPropertyKey dampedLeastSquaresLambda = keys.addDoubleKey("Damped least squares lambda");
   public static final DoubleStoredPropertyKey minimumRegionOverlapDistance = keys.addDoubleKey("Minimum Region Overlap Distance");
   public static final DoubleStoredPropertyKey maximumPointProjectionDistance = keys.addDoubleKey("Maximum Point Projection Distance");

   public PlanarRegionSLAMParameters()
   {
      this("");
   }

   public PlanarRegionSLAMParameters(String fileNameSuffix)
   {
      super(keys,
            PlanarRegionSLAMParameters.class, fileNameSuffix);

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

   public int getIterationsForMatching()
   {
      return get(iterationsForMatching);
   }

   public void setIterationsForMatching(int iterationsForMatching)
   {
      set(PlanarRegionSLAMParameters.iterationsForMatching, iterationsForMatching);
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

   public double getMinimumRegionOverlapDistance()
   {
      return get(minimumRegionOverlapDistance);
   }

   public void setMinimumRegionOverlapDistance(double minimumRegionOverlapDistance)
   {
      set(PlanarRegionSLAMParameters.minimumRegionOverlapDistance, minimumRegionOverlapDistance);
   }

   public double getMaximumPointProjectionDistance()
   {
      return get(maximumPointProjectionDistance);
   }

   public void setMaximumPointProjectionDistance(double maximumPointProjectionDistance)
   {
      set(PlanarRegionSLAMParameters.maximumPointProjectionDistance, maximumPointProjectionDistance);
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();
      LogTools.info(planarRegionSLAMParameters.getMaximumPointProjectionDistance());
      planarRegionSLAMParameters.save();
      PlanarRegionSLAMParameters planarRegionSLAMParametersForLiveMap = new PlanarRegionSLAMParameters("ForLiveMap");
      LogTools.info(planarRegionSLAMParametersForLiveMap.getMaximumPointProjectionDistance());
      planarRegionSLAMParametersForLiveMap.save();
   }
}
