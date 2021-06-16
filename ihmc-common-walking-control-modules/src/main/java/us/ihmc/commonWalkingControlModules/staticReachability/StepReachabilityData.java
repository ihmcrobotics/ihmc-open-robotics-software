package us.ihmc.commonWalkingControlModules.staticReachability;

import java.util.HashMap;
import java.util.Map;

public class StepReachabilityData
{
   private double xyzSpacing;
   private double gridSizeYaw;
   private int yawDivisions;
   private Map<StepReachabilityLatticePoint, Double> reachabilityData = new HashMap<>();

   public Map<StepReachabilityLatticePoint, Double> getLegReachabilityMap()
   {
      return reachabilityData;
   }

   public void setLegReachabilityMap(Map<StepReachabilityLatticePoint, Double> reachabilityData)
   {
      this.reachabilityData = reachabilityData;
   }

   public void setGridData(double xyzSpacing, double gridSizeYaw, int yawDivisions)
   {
      this.xyzSpacing = xyzSpacing;
      this.gridSizeYaw = gridSizeYaw;
      this.yawDivisions = yawDivisions;
   }

   public double getXyzSpacing()
   {
      return xyzSpacing;
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
