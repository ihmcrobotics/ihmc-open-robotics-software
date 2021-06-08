package us.ihmc.commonWalkingControlModules.staticReachability;

import java.util.HashMap;
import java.util.Map;

public class StepReachabilityData
{
   private double xySpacing;
   private double gridSizeYaw;
   private int yawDivisions;
   private final Map<StepReachabilityLatticePoint, Double> reachabilityData = new HashMap<>();

   public Map<StepReachabilityLatticePoint, Double> getLegReachabilityMap()
   {
      return reachabilityData;
   }

   public void setGridData(double xySpacing, double gridSizeYaw, int yawDivisions)
   {
      this.xySpacing = xySpacing;
      this.gridSizeYaw = gridSizeYaw;
      this.yawDivisions = yawDivisions;
   }

   public double getXySpacing()
   {
      return xySpacing;
   }

   public double getGridSizeYaw()
   {
      return gridSizeYaw;
   }

   public int getYawDivisions()
   {
      return yawDivisions;
   }

}
