package us.ihmc.robotics.math.interpolators;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class OrientationInterpolationCalculator
{
   // This calculator needs to be instantiated to create the following variables storing intermediate results.
   private final Quat4d startRotationQuaternion = new Quat4d();
   private final Quat4d endRotationQuaternion = new Quat4d();
   private final Quat4d relativeRotationQuaternion = new Quat4d();

   private final Vector3d angularVelocity = new Vector3d();

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   public OrientationInterpolationCalculator()
   {
   }

   /**
    * Computes the angular velocity for an interpolation between two orientations using the SLERP method.
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDot the interpolation rate
    * @return the angular velocity of the interpolated frame, w.r.t. the startOrientation, expressed in the frame in which the orientations were expressed
    */
   public void computeAngularVelocity(FrameVector angularVelocityToPack, FrameOrientation startOrientation, FrameOrientation endOrientation, double alphaDot)
   {
      startOrientation.checkReferenceFrameMatch(endOrientation);
      ReferenceFrame frame = startOrientation.getReferenceFrame();

      startOrientation.getQuaternion(startRotationQuaternion);
      endOrientation.getQuaternion(endRotationQuaternion);

      computeAngularVelocity(angularVelocity, startRotationQuaternion, endRotationQuaternion, alphaDot);

      angularVelocityToPack.setIncludingFrame(frame, angularVelocity);
   }

   /**
    * Computes the angular velocity for an interpolation between two orientations using the SLERP method.
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDot the interpolation rate
    * @return the angular velocity of the interpolated frame, w.r.t. the startOrientation, expressed in the frame in which the orientations were expressed
    */
   public void computeAngularVelocity(YoFrameVector angularVelocityToPack, YoFrameQuaternion startOrientation, YoFrameQuaternion endOrientation, double alphaDot)
   {
      angularVelocityToPack.checkReferenceFrameMatch(startOrientation);
      startOrientation.checkReferenceFrameMatch(endOrientation);

      startOrientation.get(startRotationQuaternion);
      endOrientation.get(endRotationQuaternion);

      computeAngularVelocity(angularVelocity, startRotationQuaternion, endRotationQuaternion, alphaDot);

      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Computes the angular acceleration for an interpolation between two orientations using the SLERP method.
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDoubleDot the interpolation acceleration
    * @return the angular acceleration of the interpolated frame, w.r.t. the startOrientation, expressed in the interpolated reference frame.
    */
   public void computeAngularAcceleration(FrameVector angularAccelerationToPack, FrameOrientation startOrientation, FrameOrientation endOrientation, double alphaDoubleDot)
   {
      computeAngularVelocity(angularAccelerationToPack, startOrientation, endOrientation, alphaDoubleDot); // it's really the same computation...
   }

   /**
    * Computes the angular acceleration for an interpolation between two orientations using the SLERP method.
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDoubleDot the interpolation acceleration
    * @return the angular acceleration of the interpolated frame, w.r.t. the startOrientation, expressed in the interpolated reference frame.
    */
   public void computeAngularAcceleration(YoFrameVector angularAccelerationToPack, YoFrameQuaternion startOrientation, YoFrameQuaternion endOrientation, double alphaDoubleDot)
   {
      computeAngularVelocity(angularAccelerationToPack, startOrientation, endOrientation, alphaDoubleDot); // it's really the same computation...
   }

   private void computeAngularVelocity(Vector3d angularVelocityToPack, Quat4d startRotationQuaternion, Quat4d endRotationQuaternion, double alphaDot)
   {
      if (quaternionCalculus.dot(startRotationQuaternion, endRotationQuaternion) < 0.0)
         endRotationQuaternion.negate();

      // compute relative orientation: orientation of interpolated frame w.r.t. start frame
      relativeRotationQuaternion.set(startRotationQuaternion); // R_W_S: orientation of start w.r.t. world
      relativeRotationQuaternion.conjugate(); // R_S_W: orientation of world w.r.t. start
      relativeRotationQuaternion.mul(endRotationQuaternion); // R_S_I = R_S_W * R_W_I: orientation of interpolated w.r.t. start

      quaternionCalculus.log(relativeRotationQuaternion, angularVelocityToPack);
      angularVelocityToPack.scale(alphaDot);
      quaternionCalculus.transform(endRotationQuaternion, angularVelocityToPack);
   }
}
