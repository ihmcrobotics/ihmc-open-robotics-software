package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public abstract class PlanarRegionEnvironmentInterface implements CommonAvatarEnvironmentInterface
{
   protected PlanarRegionsListDefinedEnvironment environment;
   protected final PlanarRegionsListGenerator generator;

   protected boolean hasBeenGenerated;

   private final ArrayList<PlanarRegionsList> planarRegionsLists = new ArrayList<>();
   private final ArrayList<AppearanceDefinition> appearances = new ArrayList<>();

   public PlanarRegionEnvironmentInterface()
   {
      hasBeenGenerated = false;
      generator = new PlanarRegionsListGenerator();
   }
   
   public void generateEnvironment()
   {
      String name = getClass().getSimpleName();
      PlanarRegionsList[] planarRegionsLists = new PlanarRegionsList[this.planarRegionsLists.size()];
      AppearanceDefinition[] appearances = new AppearanceDefinition[this.appearances.size()];

      for (int i = 0; i < planarRegionsLists.length; i++)
      {
         planarRegionsLists[i] = this.planarRegionsLists.get(i);
         appearances[i] = this.appearances.get(i);
      }

      environment = new PlanarRegionsListDefinedEnvironment(name, planarRegionsLists, appearances, 1e-2, false);
      hasBeenGenerated = true;
   }

   protected void addPlanarRegionsToTerrain(AppearanceDefinition appearance)
   {
      planarRegionsLists.add(generator.getPlanarRegionsList().copy());
      appearances.add(appearance);
      generator.reset();
   }

   public CombinedTerrainObject3D getCombinedTerrainObject3D()
   {
      ensureHasBeenGenerated();
      
      return environment.getCombinedTerrainObject3D();
   }
   
   public PlanarRegionsList getPlanarRegionsList()
   {
      ensureHasBeenGenerated();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      for (int i = 0; i < planarRegionsLists.size(); i++)
      {
         planarRegionsList.getPlanarRegionsAsList().addAll(planarRegionsLists.get(i).getPlanarRegionsAsList());
      }

      return planarRegionsList;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      ensureHasBeenGenerated();
      
      return environment.getTerrainObject3D();
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      ensureHasBeenGenerated();
      
      return environment.getEnvironmentRobots();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ensureHasBeenGenerated();
      
      environment.createAndSetContactControllerToARobot();
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      ensureHasBeenGenerated();
      
      environment.addContactPoints(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      ensureHasBeenGenerated();
      
      environment.addSelectableListenerToSelectables(selectedListener);
   }
   
   protected void checkHasNotBeenGenerated()
   {
      if (hasBeenGenerated)
      {
         throw new RuntimeException("Environment has already been generated!");
      }
   }
   
   private void ensureHasBeenGenerated()
   {
      if (!hasBeenGenerated)
      {
         generateEnvironment();
      }
   }
}
