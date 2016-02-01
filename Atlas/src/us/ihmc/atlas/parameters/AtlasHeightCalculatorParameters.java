package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;

/**
 * Created by agrabertilton on 3/10/15.
 */
public class AtlasHeightCalculatorParameters extends HeightCalculatorParameters
{
   public AtlasHeightCalculatorParameters(){
      super(0.1, 0.1, 0.2, 0.1, 10);
   }
}
