package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import georegression.struct.se.Se3_F64;

public interface GeoregressionTransformProvider
{
   public void packTransform(Se3_F64 transformToPack);
}
