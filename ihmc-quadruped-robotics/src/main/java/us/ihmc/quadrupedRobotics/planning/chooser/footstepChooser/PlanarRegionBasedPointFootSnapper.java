package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionBasedPointFootSnapper implements PointFootSnapper
{
   private PlanarRegionsList planarRegionsList;

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public double snapStep(double xPosition, double yPosition)
   {
      List<PlanarRegion> intersectingRegions = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(xPosition, yPosition);
      if(intersectingRegions == null)
      {
         return 0.0;
      }
      else
      {
         double maxZ = Double.NEGATIVE_INFINITY;
         for (int i = 0; i < intersectingRegions.size(); i++)
         {
            double regionHeight = intersectingRegions.get(i).getPlaneZGivenXY(xPosition, yPosition);
            if(regionHeight > maxZ)
            {
               maxZ = regionHeight;
            }
         }
         return maxZ;
      }
   }
}
