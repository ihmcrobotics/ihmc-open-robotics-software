package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.List;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class WallWorld implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D terrain = new CombinedTerrainObject3D("WallWorld");
   public WallWorld()
   {
//     terrain.addBox(-5, 0.75, 5, 2, 4);
//     terrain.addRotatedRamp(0.0, 0.7, 0.2, 1.0, 4.0, 90.0, YoAppearance.DarkBlue());
      
      setupWallSlightlyInFront();
      setupGroundPlane();
   }

   private void setupWallSlightlyInFront()
   {
      double xStart = 0.6;
      double wallWidth = 1.2;
      
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      terrain.addBox(xStart, -wallWidth/2.0, xStart + 0.1, wallWidth/2.0, 0.5, 1.8, appearance);
   }

   private void setupGroundPlane()
   {
      terrain.addBox(-10, -10, 10, 10, -0.1, 0.0, YoAppearance.Blue());
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return null;
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
