package us.ihmc.robotEnvironmentAwareness.planarRegion;

public class SurfaceNormalFilterParameters
{
   private boolean useSurfaceNormalFilter;
   private double surfaceNormalUpperBound;
   private double surfaceNormalLowerBound;
   
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
      surfaceNormalUpperBound = other.surfaceNormalUpperBound;
      surfaceNormalLowerBound = other.surfaceNormalLowerBound;
   }
   
   public void setDefaultParameters()
   {
      useSurfaceNormalFilter = true;
      surfaceNormalUpperBound = 0.7854;
      surfaceNormalLowerBound = -0.7854;
   }

   public boolean isUseSurfaceNormalFilter()
   {
      return useSurfaceNormalFilter;
   }

   public double getSurfaceNormalUpperBound()
   {
      return surfaceNormalUpperBound;
   }

   public double getSurfaceNormalLowerBound()
   {
      return surfaceNormalLowerBound;
   }

   public void setUseSurfaceNormalFilter(boolean useSurfaceNormalFilter)
   {
      this.useSurfaceNormalFilter = useSurfaceNormalFilter;
   }

   public void setSurfaceNormalUpperBound(double surfaceNormalUpperBound)
   {
      this.surfaceNormalUpperBound = surfaceNormalUpperBound;
   }

   public void setSurfaceNormalLowerBound(double surfaceNormalLowerBound)
   {
      this.surfaceNormalLowerBound = surfaceNormalLowerBound;
   }
}
