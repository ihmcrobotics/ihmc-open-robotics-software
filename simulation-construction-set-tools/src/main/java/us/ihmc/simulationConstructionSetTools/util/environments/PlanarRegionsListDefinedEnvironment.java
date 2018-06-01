package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.PlanarRegionTerrainObject;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionsListDefinedEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final String environmentName;
   private final PlanarRegionsList[] planarRegionsLists;
   private final AppearanceDefinition[] appearances;

   public PlanarRegionsListDefinedEnvironment(String environmentName, PlanarRegionsList planarRegionsList, double allowablePenetrationThickness, boolean generateGroundPlane)
   {
      this(environmentName, new PlanarRegionsList[]{planarRegionsList}, null, allowablePenetrationThickness, generateGroundPlane);
   }

   public PlanarRegionsListDefinedEnvironment(String environmentName, PlanarRegionsList[] planarRegionsList, AppearanceDefinition[] appearances, double allowablePenetrationThickness, boolean generateGroundPlane)
   {
      this.environmentName = environmentName;
      this.planarRegionsLists = planarRegionsList;

      if(appearances == null || appearances.length != planarRegionsList.length)
      {
         this.appearances = null;
      }
      else
      {
         this.appearances = appearances;
      }

      combinedTerrainObject = createCombinedTerrainObjectFromPlanarRegionsList(this.environmentName, allowablePenetrationThickness);

      if (generateGroundPlane)
      {
         combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));
      }
   }

   public PlanarRegionsListDefinedEnvironment(PlanarRegionsList planarRegionsList, double allowablePenetrationThickness, boolean generateGroundPlane)
   {
      this(PlanarRegionsListDefinedEnvironment.class.getSimpleName(), planarRegionsList, allowablePenetrationThickness, generateGroundPlane);
   }

   public PlanarRegionsListDefinedEnvironment(String environmentName, PlanarRegionsList planarRegionsList, double allowablePenetrationThickness)
   {
      this(environmentName, planarRegionsList, allowablePenetrationThickness,  true);
   }

   public PlanarRegionsListDefinedEnvironment(PlanarRegionsList planarRegionsList, double allowablePenetrationThickness)
   {
      this(planarRegionsList, allowablePenetrationThickness, true);
   }

   private CombinedTerrainObject3D createCombinedTerrainObjectFromPlanarRegionsList(String environmentName, double allowablePenetrationThickness)
   {
      CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(environmentName);

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

      return combinedTerrainObject3D;
   }

   public CombinedTerrainObject3D getCombinedTerrainObject3D()
   {
      return combinedTerrainObject;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      PrintTools.warn(this, "Environment robots currently unimplemented for this class");
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      PrintTools.warn(this, "Contact points currently unimplemented for this class");
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      PrintTools.warn(this, "Contact points currently unimplemented for this class");
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {

   }
}
