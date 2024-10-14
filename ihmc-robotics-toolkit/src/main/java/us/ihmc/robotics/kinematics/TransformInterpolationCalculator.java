package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.commons.MathTools;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 4/25/13
 * Time: 3:42 PM
 * To change this template use File | Settings | File Templates.
 */
public class TransformInterpolationCalculator
{
   private TransformInterpolationCalculator()
   {
   }

   public static void interpolate(TimeStampedTransform3D timeStampedTransform1, TimeStampedTransform3D timeStampedTransform2, TimeStampedTransform3D resultToPack, long timeStamp)
   {
      long timeStamp1 = timeStampedTransform1.getTimeStamp();
      long timeStamp2 = timeStampedTransform2.getTimeStamp();

      MathTools.checkIntervalContains(timeStamp, timeStamp1, timeStamp2);

      RigidBodyTransform transform1 = timeStampedTransform1.getTransform3D();
      RigidBodyTransform transform2 = timeStampedTransform2.getTransform3D();

      resultToPack.setTimeStamp(timeStamp);

      double alpha = ((double) (timeStamp - timeStamp1)) / ((double) (timeStamp2 - timeStamp1));
      resultToPack.getTransform3D().interpolate(transform1, transform2, alpha);
   }
}
