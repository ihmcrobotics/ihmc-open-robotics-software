package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class WallAtDistanceEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private final double wallDistance;

   /**
    * This world is for lidar tests.
    *
    * @param wallDistance
    */
   public WallAtDistanceEnvironment(double wallDistance)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

      this.wallDistance = wallDistance;
      
      combinedTerrainObject.addTerrainObject(setupWall("WallyTheWall"));
      combinedTerrainObject.addTerrainObject(setupGround());
   }
   
   private CombinedTerrainObject3D setupWall(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      double xStart = wallDistance;
      double wallMinY = -1000;
      double wallMaxY = 1000;
      double wallZStart = -1000;
      double wallZEnd = 1000;
      
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      combinedTerrainObject.addBox(xStart, wallMinY, xStart + 0.1, wallMaxY, wallZStart, wallZEnd, appearance);
      return combinedTerrainObject;
   }
   
   private CombinedTerrainObject3D setupGround()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("Ground");
      combinedTerrainObject.addBox(-0.2, -0.5, 0.2, 0.5, -0.05, 0.0, YoAppearance.Gold());      
      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      
   }
}
