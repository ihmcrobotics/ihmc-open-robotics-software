package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import georegression.struct.se.Se3_F64;

public interface GeoregressionTransformListener
{
   public void handleTransform(Se3_F64 transform);
}
