package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.PlanarRegionTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionsListDefinedEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final String environmentName;
   private final PlanarRegionsList planarRegionsList;

   public PlanarRegionsListDefinedEnvironment(String environmentName, PlanarRegionsList planarRegionsList, boolean generateGroundPlane)
   {
      this.environmentName = environmentName;
      this.planarRegionsList = planarRegionsList;

      combinedTerrainObject = createCombinedTerrainObjectFromPlanarRegionsList(this.environmentName, this.planarRegionsList);

      if (generateGroundPlane)
      {
         combinedTerrainObject.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("Ground"));

      }
   }

   public PlanarRegionsListDefinedEnvironment(PlanarRegionsList planarRegionsList, boolean generateGroundPlane)
   {
      this(PlanarRegionsListDefinedEnvironment.class.getSimpleName(), planarRegionsList, generateGroundPlane);
   }

   public PlanarRegionsListDefinedEnvironment(String environmentName, PlanarRegionsList planarRegionsList)
   {
      this(environmentName, planarRegionsList, true);
   }

   public PlanarRegionsListDefinedEnvironment(PlanarRegionsList planarRegionsList)
   {
      this(planarRegionsList, true);
   }

   private CombinedTerrainObject3D createCombinedTerrainObjectFromPlanarRegionsList(String environmentName, PlanarRegionsList planarRegionsList)
   {
      CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(environmentName);

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         combinedTerrainObject3D.addTerrainObject(new PlanarRegionTerrainObject(planarRegion));
      }

      return combinedTerrainObject3D;
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
