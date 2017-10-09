package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import java.util.List;

import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public abstract class PlanarRegionEnvironmentInterface implements CommonAvatarEnvironmentInterface
{
   protected PlanarRegionsListDefinedEnvironment environment;
   protected final PlanarRegionsListGenerator generator;
   
   private boolean hasBeenGenerated;
   
   public PlanarRegionEnvironmentInterface()
   {
      hasBeenGenerated = false;
      generator = new PlanarRegionsListGenerator();
   }
   
   public void generateEnvironment()
   {      
      environment = new PlanarRegionsListDefinedEnvironment(generator.getPlanarRegionsList(), 1e-2, false);
      hasBeenGenerated = true;
   }

   public CombinedTerrainObject3D getCombinedTerrainObject3D()
   {
      ensureHasBeenGenerated();
      
      return environment.getCombinedTerrainObject3D();
   }
   
   public PlanarRegionsList getPlanarRegionsList()
   {
      ensureHasBeenGenerated();
      
      return generator.getPlanarRegionsList();
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
