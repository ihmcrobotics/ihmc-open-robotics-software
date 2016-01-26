package us.ihmc.aware.parameters;

import us.ihmc.aware.parameters.QuadrupedStandPrepParameters;

public class DefaultQuadrupedStandPrepParameters implements QuadrupedStandPrepParameters
{
   @Override
   public double getTrajectoryTime()
   {
      return 1.0;
   }
}
