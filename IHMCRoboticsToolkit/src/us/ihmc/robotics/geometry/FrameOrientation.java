package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameOrientation extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final Quat4d quaternion = new Quat4d();
   private final Quat4d tempQuaternionForTransform = new Quat4d();

   public FrameOrientation(FrameOrientation orientation)
   {
      referenceFrame = orientation.referenceFrame;
      quaternion.set(orientation.quaternion);
   }

   public FrameOrientation(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      quaternion.set(0.0, 0.0, 0.0, 1.0);
   }

   public FrameOrientation()
   {
      referenceFrame = ReferenceFrame.getWorldFrame();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, RigidBodyTransform transform3D)
   {
      this.referenceFrame = referenceFrame;
      transform3D.get(quaternion);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4d quaternion)
   {
      this.referenceFrame = referenceFrame;
      this.quaternion.set(quaternion);
      this.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4f quaternion)
   {
      this.referenceFrame = referenceFrame;
      this.quaternion.set(quaternion);
      this.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this.referenceFrame = referenceFrame;

      quaternion.set(qx, qy, qz, qs);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      this.referenceFrame = referenceFrame;
      setYawPitchRoll(yaw, pitch, roll);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      this.referenceFrame = referenceFrame;
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Matrix3d rotation)
   {
      this.referenceFrame = referenceFrame;
      RotationTools.convertMatrixToQuaternion(rotation, quaternion);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, AxisAngle4d orientation)
   {
      this.referenceFrame = referenceFrame;
      quaternion.set(orientation);
      normalize();
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame)
   {
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, RandomTools.generateRandomQuaternion(random));
      return randomOrientation;
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame, double yawMin, double yawMax, double pitchMin, double pitchMax, double rollMin, double rollMax)
   {
      double yaw = RandomTools.generateRandomDouble(random, yawMin, yawMax);
      double pitch = RandomTools.generateRandomDouble(random, pitchMin, pitchMax);
      double roll = RandomTools.generateRandomDouble(random, rollMin, rollMax);
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, yaw, pitch, roll);
      return randomOrientation;
   }

   public void set(AxisAngle4d axisAngle4d)
   {
      quaternion.set(axisAngle4d);
      normalize();
   }

   public void set(Quat4d quat4d)
   {
      quaternion.set(quat4d);
      normalize();
   }

   public void set(Matrix3d rotationMatrix)
   {
      RotationTools.convertMatrixToQuaternion(rotationMatrix, quaternion);
      normalize();
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      if (Double.isNaN(roll))
      {
         throw new RuntimeException("Orientation.setYawPitchRoll(). yaw = " + yaw + ", pitch = " + pitch + ", roll = " + roll);
      }

      RotationTools.convertYawPitchRollToQuaternion(yaw, pitch, roll, quaternion);
      normalize();
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void set(FrameOrientation orientation)
   {
      referenceFrame.checkReferenceFrameMatch(orientation.referenceFrame);
      quaternion.set(orientation.quaternion);
      normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4d quat)
   {
      this.referenceFrame = referenceFrame;
      quaternion.set(quat);
      normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, AxisAngle4d axisAngle)
   {
      this.referenceFrame = referenceFrame;
      quaternion.set(axisAngle);
      normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3d rotationMatrix)
   {
      this.referenceFrame = referenceFrame;
      RotationTools.convertMatrixToQuaternion(rotationMatrix, quaternion);
      normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform transform3D)
   {
      this.referenceFrame = referenceFrame;
      transform3D.get(quaternion);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      setIncludingFrame(referenceFrame, yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      this.referenceFrame = referenceFrame;
      this.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setIncludingFrame(FrameOrientation orientation)
   {
      referenceFrame = orientation.referenceFrame;
      quaternion.set(orientation.quaternion);
      normalize();
   }

   // TODO Find a better. I chose setToZero() as in FrameTuple.
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      quaternion.set(0.0, 0.0, 0.0, 1.0);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void getQuaternion(Quat4d quat4d)
   {
      quat4d.set(quaternion);
   }

   public void getMatrix3d(Matrix3d matrixToPack)
   {
      matrixToPack.set(quaternion);
   }

   public void getAxisAngle(AxisAngle4d axisAngleToPack)
   {
      axisAngleToPack.set(quaternion);
   }

   public void getTransform3D(RigidBodyTransform transformToPack)
   {
      transformToPack.setRotation(quaternion);
   }

   public void getYawPitchRoll(double[] yawPitchRoll)
   {
      RotationTools.convertQuaternionToYawPitchRoll(quaternion, yawPitchRoll);
   }

   public double[] getYawPitchRoll()
   {
      double[] yawPitchRollToReturn = new double[3];
      getYawPitchRoll(yawPitchRollToReturn);
      return yawPitchRollToReturn;
   }

   public double getYaw()
   {
      return RotationTools.computeYaw(quaternion);
   }

   public double getPitch()
   {
      return RotationTools.computePitch(quaternion);
   }

   public double getRoll()
   {
      return RotationTools.computeRoll(quaternion);
   }

   public Quat4d getQuaternionCopy()
   {
      return new Quat4d(quaternion);
   }

   public Matrix3d getMatrix3dCopy()
   {
      Matrix3d ret = new Matrix3d();
      ret.set(quaternion);

      return ret;
   }

   public void getFrameOrientation2dIncludingFrame(FrameOrientation2d frameOrientation2dToPack)
   {
      frameOrientation2dToPack.setIncludingFrame(referenceFrame, getYaw());
   }

   public void applyTransform(RigidBodyTransform transform3D)
   {
      transform3D.get(tempQuaternionForTransform);
      quaternion.mul(tempQuaternionForTransform, quaternion);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
      {
         return;
      }

      referenceFrame.verifySameRoots(desiredFrame);
      RigidBodyTransform referenceTf, desiredTf;

      if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
      {
         applyTransform(referenceTf);
      }

      if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
      {
         applyTransform(desiredTf);
      }

      referenceFrame = desiredFrame;
      normalize();
   }

   public void interpolate(FrameOrientation orientationOne, FrameOrientation orientationTwo, double alpha)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
      quaternion.interpolate(orientationOne.quaternion, orientationTwo.quaternion, alpha);
      referenceFrame = orientationOne.getReferenceFrame();
   }

   public void setOrientationFromOneToTwo(FrameOrientation orientationOne, FrameOrientation orientationTwo)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      this.checkReferenceFrameMatch(orientationOne);

      this.quaternion.conjugate(orientationTwo.quaternion);
      this.quaternion.mul(orientationOne.quaternion);
   }

   /**
    * Normalize the quaternion and also limits the described angle magnitude in {-Pi, Pi].
    * The latter prevents some controllers to poop their pants.
    */
   public void normalize()
   {
      quaternion.normalize();
      if (quaternion.getW() < 0.0)
         quaternion.negate();
   }

   public boolean epsilonEquals(FrameOrientation frameOrientation, double epsilon)
   {
      boolean referenceFramesMatch = referenceFrame == frameOrientation.referenceFrame;
      boolean quaternionsAreEqual = RotationTools.quaternionEpsilonEquals(quaternion, frameOrientation.quaternion, epsilon);

      return referenceFramesMatch && quaternionsAreEqual;
   }

   @Override
   public String toString()
   {
      String stringToReturn = "";

      stringToReturn = stringToReturn + toStringAsYawPitchRoll() + "\n";
      stringToReturn = stringToReturn + toStringAsQuaternion();

      return stringToReturn;
   }

   public String toStringAsYawPitchRoll()
   {
      double[] yawPitchRoll = getYawPitchRoll();

      return "yaw-pitch-roll: (" + yawPitchRoll[0] + ", " + yawPitchRoll[1] + ", " + yawPitchRoll[2] + ")-" + referenceFrame.getName();
   }

   public String toStringAsQuaternion()
   {
      return "quaternion: " + quaternion + "-" + referenceFrame.getName();
   }
}
