package us.ihmc.robotEnvironmentAwareness.planarRegion;

public class SurfaceNormalFilterParameters
{
   private boolean useSurfaceNormalFilter;
   private double surfaceNormalUpperBoundDegree;
   private double surfaceNormalLowerBoundDegree;
   
   public SurfaceNormalFilterParameters()
   {
      setDefaultParameters();
   }
   
   public SurfaceNormalFilterParameters(SurfaceNormalFilterParameters other)
   {
      set(other);
   }
   
   public void set(SurfaceNormalFilterParameters other)
   {
      useSurfaceNormalFilter = other.useSurfaceNormalFilter;
      surfaceNormalUpperBoundDegree = other.surfaceNormalUpperBoundDegree;
      surfaceNormalLowerBoundDegree = other.surfaceNormalLowerBoundDegree;
   }
   
   public void setDefaultParameters()
   {
      useSurfaceNormalFilter = false;
      surfaceNormalUpperBoundDegree = 45.0;
      surfaceNormalLowerBoundDegree = -45.0;
   }

   public boolean isUseSurfaceNormalFilter()
   {
      return useSurfaceNormalFilter;
   }

   public double getSurfaceNormalUpperBoundDegree()
   {
      return surfaceNormalUpperBoundDegree;
   }

   public double getSurfaceNormalLowerBoundDegree()
   {
      return surfaceNormalLowerBoundDegree;
   }

   public void setUseSurfaceNormalFilter(boolean useSurfaceNormalFilter)
   {
      this.useSurfaceNormalFilter = useSurfaceNormalFilter;
   }

   public void setSurfaceNormalUpperBoundDegree(double surfaceNormalUpperBoundDegree)
   {
      this.surfaceNormalUpperBoundDegree = surfaceNormalUpperBoundDegree;
   }

   public void setSurfaceNormalLowerBoundDegree(double surfaceNormalLowerBoundDegree)
   {
      this.surfaceNormalLowerBoundDegree = surfaceNormalLowerBoundDegree;
   }
}
