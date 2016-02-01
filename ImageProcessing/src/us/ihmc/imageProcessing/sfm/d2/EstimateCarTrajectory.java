package us.ihmc.imageProcessing.sfm.d2;

import georegression.metric.UtilAngle;
import georegression.struct.se.Se2_F64;

/**
 * Estimates the vehicles turn rate from 2D motions.  turn rate = change angle / line integral
 *
 * @author Peter Abeles
 */
public class EstimateCarTrajectory
{
   double minDistance;
   Se2_F64 history[];
   int start = 0;
   int total = 0;

   double turnRate;

   public EstimateCarTrajectory( int N , double minDistance)
   {
      history = new Se2_F64[N];
      for( int i = 0; i < N; i++ ) {
         history[i] = new Se2_F64();
      }
      this.minDistance = minDistance;
   }

   public void reset() {
      start = total = 0;
   }

   public boolean process( Se2_F64 motion ) {
      history[start].set(motion);

      if( total < history.length )
         total++;

      boolean result;

      if(total >= history.length ) {

         double line = 0;
         for( int i = 1; i < history.length; i++ ) {
            Se2_F64 a = get(i-1);
            Se2_F64 b = get(i);

            line += a.T.distance(b.T);
         }

         double yaw0 = history[start].getYaw();  // most recent
         double yaw1 = history[(start+1)% history.length].getYaw();          // oldest
         double deltaAngle = UtilAngle.distanceCCW(yaw0,yaw1);
         if( deltaAngle > Math.PI )
            deltaAngle -= 2.0*Math.PI;

         // see if there is enough motion for the results to be valid
         if( line < minDistance ) {
            result = false;
         } else {
            turnRate = deltaAngle/line;
            result = true;
         }
      } else {
         result = false;
      }

      start = (start + 1) % history.length;

      return result;
   }

   private Se2_F64 get( int which ) {
      return history[ (start+1+which)%history.length ];
   }

   public double getTurnRate()
   {
      return turnRate;
   }
}
