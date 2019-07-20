package us.ihmc.humanoidBehaviors.tools.perception;

public class PlanarRegionSLAMParameters
{
   private int iterations = 3;
   private double boundingBoxHeight = 0.1;
   private double minimumNormalDotProduct = 0.9;
 
   public PlanarRegionSLAMParameters()
   {
      
   }

   public double getBoundingBoxHeight()
   {
      return boundingBoxHeight;
   }

   public void setBoundingBoxHeight(double boundingBoxHeight)
   {
      this.boundingBoxHeight = boundingBoxHeight;
   }

   public int getIterations()
   {
      return iterations;
   }

   public void setIterations(int iterations)
   {
      this.iterations = iterations;
   }

   public double getMinimumNormalDotProduct()
   {
      return minimumNormalDotProduct;
   }

   public void setMinimumNormalDotProduct(double minimumNormalDotProduct)
   {
      this.minimumNormalDotProduct = minimumNormalDotProduct;
   }
}
