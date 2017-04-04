package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;

public class AtlasStraightLegWalkingParameters extends StraightLegWalkingParameters
{
   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }
}
