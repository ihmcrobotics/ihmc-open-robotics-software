package us.ihmc.simulationconstructionset.util.environments.planarRegionEnvironments;

import java.util.List;

import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public abstract class PlanarRegionEnvironmentInterface implements CommonAvatarEnvironmentInterface
{
   protected PlanarRegionsListDefinedEnvironment environment;
   protected final PlanarRegionsListGenerator generator;
   
   public PlanarRegionEnvironmentInterface()
   {
      generator = new PlanarRegionsListGenerator();
   }
   
   protected abstract void buildGenerator(PlanarRegionsListGenerator generator);
   
   protected void generateEnvironment()
   {
      buildGenerator(generator);
      
      environment = new PlanarRegionsListDefinedEnvironment(generator.getPlanarRegionsList(), 1e-2, false);
   }

   public CombinedTerrainObject3D getCombinedTerrainObject3D()
   {
      return environment.getCombinedTerrainObject3D();
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return environment.getTerrainObject3D();
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return environment.getEnvironmentRobots();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      environment.createAndSetContactControllerToARobot();
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      environment.addContactPoints(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      environment.addSelectableListenerToSelectables(selectedListener);
   }
}
