package us.ihmc.robotics.math.interpolators;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class OrientationInterpolationCalculator
{
   // This calculator needs to be instantiated to create the following variables storing intermediate results.
   private final Matrix3d startRotationMatrix = new Matrix3d();
   private final Matrix3d endRotationMatrix = new Matrix3d();
   private final Matrix3d relativeRotationMatrix = new Matrix3d();

   private final AxisAngle4d axisAngle = new AxisAngle4d();
   private final Vector3d angularVelocity = new Vector3d();

   public OrientationInterpolationCalculator()
   {
   }

   /**
    * Computes the angular velocity for an interpolation between two orientations
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDot the interpolation rate
    * @return the angular velocity of the interpolated frame, w.r.t. the startOrientation, expressed in the frame in which the orientations were expressed
    */
   public void computeAngularVelocity(FrameVector angularVelocityToPack, FrameOrientation startOrientation, FrameOrientation endOrientation, double alphaDot)
   {
      startOrientation.checkReferenceFrameMatch(endOrientation);
      ReferenceFrame frame = startOrientation.getReferenceFrame();

      startOrientation.getMatrix3d(startRotationMatrix);
      endOrientation.getMatrix3d(endRotationMatrix);

      computeAngularVelocity(angularVelocity, startRotationMatrix, endRotationMatrix, alphaDot);

      angularVelocityToPack.setIncludingFrame(frame, angularVelocity);
   }

   public void computeAngularVelocity(YoFrameVector angularVelocityToPack, YoFrameQuaternion startOrientation, YoFrameQuaternion endOrientation, double alphaDot)
   {
      angularVelocityToPack.checkReferenceFrameMatch(startOrientation);
      startOrientation.checkReferenceFrameMatch(endOrientation);

      startOrientation.get(startRotationMatrix);
      endOrientation.get(endRotationMatrix);

      computeAngularVelocity(angularVelocity, startRotationMatrix, endRotationMatrix, alphaDot);

      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Computes the angular acceleration for an interpolation between two orientations
    * @param startOrientation the starting orientation
    * @param endOrientation the final orientation
    * @param alphaDoubleDot the interpolation acceleration
    * @return the angular acceleration of the interpolated frame, w.r.t. the startOrientation, expressed in the interpolated reference frame.
    */
   public void computeAngularAcceleration(FrameVector angularAccelerationToPack, FrameOrientation startOrientation, FrameOrientation endOrientation,
         double alphaDoubleDot)
   {
      computeAngularVelocity(angularAccelerationToPack, startOrientation, endOrientation, alphaDoubleDot); // it's really the same computation...
   }

   public void computeAngularAcceleration(YoFrameVector angularAccelerationToPack, YoFrameQuaternion startOrientation, YoFrameQuaternion endOrientation,
         double alphaDoubleDot)
   {
      computeAngularVelocity(angularAccelerationToPack, startOrientation, endOrientation, alphaDoubleDot); // it's really the same computation...
   }

   private void computeAngularVelocity(Vector3d angularVelocityToPack, Matrix3d startRotationMatrix, Matrix3d endRotationMatrix, double alphaDot)
   {
      // compute relative orientation: orientation of interpolated frame w.r.t. start frame
      relativeRotationMatrix.set(startRotationMatrix); // R_W_S: orientation of start w.r.t. world
      relativeRotationMatrix.transpose(); // R_S_W: orientation of world w.r.t. start
      relativeRotationMatrix.mul(endRotationMatrix); // R_S_I = R_S_W * R_W_I: orientation of interpolated w.r.t. start

      // convert to axis-angle
//      axisAngle.set(relativeRotationMatrix);
      RotationTools.axisAngleFromMatrix(relativeRotationMatrix, axisAngle);
      

      // compute angular rate
      double angle = axisAngle.getAngle();
      double angleDot = alphaDot * angle;

      // compute angular velocity in frame with the same orientation as the start frame.
      // note that this is also the angular velocity in frame with same orientation as interpolated frame,
      // since the latter is the former, rotated about the angular velocity axis
      angularVelocityToPack.setX(angleDot * axisAngle.getX());
      angularVelocityToPack.setY(angleDot * axisAngle.getY());
      angularVelocityToPack.setZ(angleDot * axisAngle.getZ());

      // rotate back to the frame in which the original Orientations were expressed
      endRotationMatrix.transform(angularVelocityToPack);
   }
}
