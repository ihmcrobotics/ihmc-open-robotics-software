package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.PlanarRegionTerrainObject;

public class PlanarRegionEnvironmentTools
{
   public static void addRegionsToEnvironment(CombinedTerrainObject3D combinedTerrainObject3D, PlanarRegionsList[] planarRegionsLists, AppearanceDefinition[] appearances, double allowablePenetrationThickness)
   {
      for (int i = 0; i < planarRegionsLists.length; i++)
      {
         PlanarRegionsList planarRegionsList = planarRegionsLists[i];
         for (int j = 0; j < planarRegionsList.getNumberOfPlanarRegions(); j++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(j);

            if(appearances == null)
            {
               combinedTerrainObject3D.addTerrainObject(new PlanarRegionTerrainObject(planarRegion, allowablePenetrationThickness));
            }
            else
            {
               combinedTerrainObject3D.addTerrainObject(new PlanarRegionTerrainObject(planarRegion, allowablePenetrationThickness, appearances[i]));
            }
         }
      }
   }
}
