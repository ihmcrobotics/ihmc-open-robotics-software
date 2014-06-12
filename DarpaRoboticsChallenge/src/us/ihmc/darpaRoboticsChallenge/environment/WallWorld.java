package us.ihmc.darpaRoboticsChallenge.environment;

import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;

public class WallWorld extends CombinedTerrainObject
{

   public WallWorld()
   {
      super("WallWorld");

      addBox(-5, 0.75, 5, 2, 4);
      
      addBox(-10, -10, 10, 10, -0.1, 0.0);
   }

   
}
