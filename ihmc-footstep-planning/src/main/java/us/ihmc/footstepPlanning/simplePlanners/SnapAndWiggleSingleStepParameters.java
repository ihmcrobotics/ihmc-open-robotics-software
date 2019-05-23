package us.ihmc.footstepPlanning.simplePlanners;

public class SnapAndWiggleSingleStepParameters
{
   /**
    * If a step is wiggled in the backwards (wrt the previous step) by this amount,
    * another step is placed further out. This avoids getting stuck on the top of a ledge.
    * If this values is NaN, this check is disabled
    */
   private double wiggleInWrongDirectionThreshold = 0.04;

   /**
    * If a step is within this distance to a step-up, it will be shifted away
    */
   private double closestDistanceToCliff = 0.05;

   /**
    * Minimum step-up height to shift away from
    */
   private double cliffHeightToAvoid = 0.05;

   /**
    * Amount to wiggle inside of planar region
    */
   private double wiggleInsideDelta = 0.03;

   /**
    * Maximum angle of planar region considered for a foothold.
    * Zero degrees corresponds to a vertical normal
    */
   private double maxPlanarRegionAngle = Math.toRadians(25.0);

   /**
    * Minimum planar region area for a foothold
    */
   private double minPlanarRegionArea = 0.05;

   /**
    * Needed to calculate shift values on and away from cliffs.
    * If left as NaN shifting is disabled
    */
   private double footLength = Double.NaN;

   public double getWiggleInWrongDirectionThreshold()
   {
      return wiggleInWrongDirectionThreshold;
   }

   public void setWiggleInWrongDirectionThreshold(double wiggleInWrongDirectionThreshold)
   {
      this.wiggleInWrongDirectionThreshold = wiggleInWrongDirectionThreshold;
   }

   public double getClosestDistanceToCliff()
   {
      return closestDistanceToCliff;
   }

   public void setClosestDistanceToCliff(double closestDistanceToCliff)
   {
      this.closestDistanceToCliff = closestDistanceToCliff;
   }

   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid;
   }

   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid = cliffHeightToAvoid;
   }

   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta;
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta = wiggleInsideDelta;
   }

   public double getMaxPlanarRegionAngle()
   {
      return maxPlanarRegionAngle;
   }

   public void setMaxPlanarRegionAngle(double maxPlanarRegionAngle)
   {
      this.maxPlanarRegionAngle = maxPlanarRegionAngle;
   }

   public double getMinPlanarRegionArea()
   {
      return minPlanarRegionArea;
   }

   public void setMinPlanarRegionArea(double minPlanarRegionArea)
   {
      this.minPlanarRegionArea = minPlanarRegionArea;
   }

   public double getFootLength()
   {
      return footLength;
   }

   public void setFootLength(double footLength)
   {
      this.footLength = footLength;
   }
}
