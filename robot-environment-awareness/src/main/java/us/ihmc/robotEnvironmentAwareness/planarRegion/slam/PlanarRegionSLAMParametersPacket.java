package us.ihmc.robotEnvironmentAwareness.planarRegion.slam;

public class PlanarRegionSLAMParametersPacket
{
   private int iterationsForMatching;
   private double boundingBoxHeight;
   private double minimumNormalDotProduct;
   private double dampedLeastSquaresLambda;
   private double minimumRegionOverlapDistance;
   private double maximumPointProjectionDistance;

   public PlanarRegionSLAMParametersPacket()
   {
   }

   public PlanarRegionSLAMParametersPacket(PlanarRegionSLAMParameters parameters)
   {
      this();
      set(parameters);
   }

   public void set(PlanarRegionSLAMParameters parameters)
   {
      iterationsForMatching = parameters.getIterationsForMatching();
      boundingBoxHeight = parameters.getBoundingBoxHeight();
      minimumNormalDotProduct = parameters.getMinimumNormalDotProduct();
      dampedLeastSquaresLambda = parameters.getDampedLeastSquaresLambda();
      minimumRegionOverlapDistance = parameters.getMinimumRegionOverlapDistance();
      maximumPointProjectionDistance = parameters.getMaximumPointProjectionDistance();
   }

   public void get(PlanarRegionSLAMParameters parameters)
   {
      parameters.setIterationsForMatching(iterationsForMatching);
      parameters.setBoundingBoxHeight(boundingBoxHeight);
      parameters.setMinimumNormalDotProduct(minimumNormalDotProduct);
      parameters.setDampedLeastSquaresLambda(dampedLeastSquaresLambda);
      parameters.setMinimumRegionOverlapDistance(minimumRegionOverlapDistance);
      parameters.setMaximumPointProjectionDistance(maximumPointProjectionDistance);
   }
}