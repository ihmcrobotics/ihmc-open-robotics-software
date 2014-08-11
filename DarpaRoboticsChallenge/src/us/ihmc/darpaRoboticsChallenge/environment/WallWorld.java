package us.ihmc.darpaRoboticsChallenge.environment;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject3D;

public class WallWorld extends CombinedTerrainObject3D
{

   public WallWorld()
   {
      super("WallWorld");

//      addBox(-5, 0.75, 5, 2, 4);
//      addRotatedRamp(0.0, 0.7, 0.2, 1.0, 4.0, 90.0, YoAppearance.DarkBlue());
      
      setupWallSlightlyInFront();
      setupGroundPlane();
   }

   private void setupWallSlightlyInFront()
   {
      double xStart = 0.6;
      double wallWidth = 1.2;
      
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      addBox(xStart, -wallWidth/2.0, xStart + 0.1, wallWidth/2.0, 0.5, 1.8, appearance);
   }

   private void setupGroundPlane()
   {
      addBox(-10, -10, 10, 10, -0.1, 0.0, YoAppearance.Blue());
   }

   
}
