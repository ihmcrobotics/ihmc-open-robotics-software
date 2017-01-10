package us.ihmc.simulationconstructionset.util.environments.planarRegionEnvironments;

import java.util.List;

import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class LittleWallsWithIncreasingHeightEnvironment implements CommonAvatarEnvironmentInterface
{
   private final PlanarRegionsListDefinedEnvironment environment;
   private final PlanarRegionsListGenerator generator;
   
   public LittleWallsWithIncreasingHeightEnvironment()
   {
      generator = new PlanarRegionsListGenerator();
      
      generator.translate(2.0, 0.0, -0.01);
      generator.addCubeReferencedAtCenter(6.0, 1.0, 0.00005);
      generator.translate(-2.0, 0.0, 0.0);
      generator.translate(0.35, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.62, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.14);
      generator.translate(0.3, -0.3, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.15);
      generator.translate(0.4, 0.1, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.11);
      
      environment = new PlanarRegionsListDefinedEnvironment(generator.getPlanarRegionsList(), 1e-2, false);
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
