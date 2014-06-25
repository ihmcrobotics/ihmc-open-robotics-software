package us.ihmc.darpaRoboticsChallenge.environment;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;

public class WallWorld extends CombinedTerrainObject
{

   public WallWorld()
   {
      super("WallWorld");

//      addBox(-5, 0.75, 5, 2, 4);
      
      addRotatedRamp(0.0, 0.7, 0.2, 1.0, 4.0, 90.0, YoAppearance.DarkBlue());
      
      addBox(-10, -10, 10, 10, -0.1, 0.0);
   }

   
}
