package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 4/25/13
 * Time: 3:42 PM
 * To change this template use File | Settings | File Templates.
 */
public class TransformInterpolationCalculator
{
   private final Vector3D transform1Translation = new Vector3D();
   private final Vector3D transform2Translation = new Vector3D();
   private final Quaternion transform1Quaternion = new Quaternion();
   private final Quaternion transform2Quaternion = new Quaternion();
   private final Vector3D interpolatedTranslation = new Vector3D();
   private final Quaternion interpolatedQuaternion = new Quaternion();
   
   /**
    *        Computes the interpolation between the two transforms using the alpha parameter to control the blend.
    *        Note that the transforms must have a proper rotation matrix, meaning it satisfies: R'R = I and det(R) = 1
    * @param transform1
    * @param transform2
    * @param alpha Ranges from [0, 1], where return = (1- alpha) * tansform1 + (alpha) * transform2
    * @return  return = (1- alpha) * tansform1 + alpha * transform2
    */
   public void computeInterpolation(RigidBodyTransform transform1, RigidBodyTransform transform2, RigidBodyTransform result, double alpha)
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      
      transform1.get(transform1Quaternion, transform1Translation);
      transform2.get(transform2Quaternion, transform2Translation);
      
      interpolatedTranslation.interpolate(transform1Translation, transform2Translation, alpha);
      interpolatedQuaternion.interpolate(transform1Quaternion, transform2Quaternion, alpha);
      
      result.setRotationAndZeroTranslation(interpolatedQuaternion);
      result.setTranslation(interpolatedTranslation);
   }

   public void interpolate(TimeStampedTransform3D timeStampedTransform1, TimeStampedTransform3D timeStampedTransform2, TimeStampedTransform3D resultToPack, long timeStamp)
   {
      long timeStamp1 = timeStampedTransform1.getTimeStamp();
      long timeStamp2 = timeStampedTransform2.getTimeStamp();

      MathTools.checkIfInRange(timeStamp, timeStamp1, timeStamp2);

      RigidBodyTransform transform1 = timeStampedTransform1.getTransform3D();
      RigidBodyTransform transform2 = timeStampedTransform2.getTransform3D();

      resultToPack.setTimeStamp(timeStamp);

      double alpha = ((double) (timeStamp - timeStamp1)) / ((double) (timeStamp2 - timeStamp1));
      RigidBodyTransform interpolatedTransform = resultToPack.getTransform3D();
      computeInterpolation(transform1, transform2, interpolatedTransform, alpha);
   }
}
