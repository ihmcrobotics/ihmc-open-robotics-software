package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import georegression.struct.se.Se3_F64;

public class GeoregressionTransformListenerAndProvider implements GeoregressionTransformListener, GeoregressionTransformProvider
{
   private Se3_F64 transform = new Se3_F64();

   public synchronized void packTransform(Se3_F64 transformToPack)
   {
      transformToPack.set(this.transform);
   }

   public synchronized void handleTransform(Se3_F64 transform)
   {
      this.transform.set(transform);
   }
}
