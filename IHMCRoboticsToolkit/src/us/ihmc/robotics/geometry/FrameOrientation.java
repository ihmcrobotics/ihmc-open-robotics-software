package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
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
   private final Matrix3d tempMatrixForYawPitchRollConversion = new Matrix3d();

   public FrameOrientation(FrameOrientation orientation)
   {
      referenceFrame = orientation.referenceFrame;
      quaternion.set(orientation.quaternion);
   }

   public FrameOrientation(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FrameOrientation()
   {
      setToZero(ReferenceFrame.getWorldFrame());
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

   public void set(Quat4d quaternion)
   {
      this.quaternion.set(quaternion);
      normalize();
   }

   public void set(double qx, double qy, double qz, double qs)
   {
      quaternion.set(qx, qy, qz, qs);
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

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4d quaternion)
   {
      this.referenceFrame = referenceFrame;
      set(quaternion);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this.referenceFrame = referenceFrame;
      set(qx, qy, qz, qs);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, AxisAngle4d axisAngle)
   {
      this.referenceFrame = referenceFrame;
      set(axisAngle);
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
      setToZero();
   }

   // TODO Find a better. I chose setToZero() as in FrameTuple.
   public void setToZero()
   {
      quaternion.set(0.0, 0.0, 0.0, 1.0);
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
   }

   public void setToNaN()
   {
      quaternion.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   public boolean containsNaN()
   {
      return Double.isNaN(quaternion.x) || Double.isNaN(quaternion.y) || Double.isNaN(quaternion.z) || Double.isNaN(quaternion.w);
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

   public void getMatrix3f(Matrix3f matrixToPack)
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

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      tempMatrixForYawPitchRollConversion.set(quaternion);
      yawPitchRollToPack[0] = Math.atan2(tempMatrixForYawPitchRollConversion.m10, tempMatrixForYawPitchRollConversion.m00);

      if (Math.abs(tempMatrixForYawPitchRollConversion.m20) < 1.0 - 1e-10)
         yawPitchRollToPack[1] = Math.asin(-tempMatrixForYawPitchRollConversion.m20);
      else
         yawPitchRollToPack[1] = -Math.signum(tempMatrixForYawPitchRollConversion.m20) * Math.PI / 2.0;

      yawPitchRollToPack[2] = Math.atan2(tempMatrixForYawPitchRollConversion.m21, tempMatrixForYawPitchRollConversion.m22);

      if (Double.isNaN(yawPitchRollToPack[0]) || Double.isNaN(yawPitchRollToPack[1]) || Double.isNaN(yawPitchRollToPack[2]))
      {
         throw new RuntimeException("yaw, pitch, or roll are NaN! rotationMatrix = " + tempMatrixForYawPitchRollConversion);
      }
   }

   public double[] getYawPitchRoll()
   {
      double[] yawPitchRollToReturn = new double[3];
      getYawPitchRoll(yawPitchRollToReturn);
      return yawPitchRollToReturn;
   }

   public double getYaw()
   {
      tempMatrixForYawPitchRollConversion.set(quaternion);
      return RotationTools.computeYaw(tempMatrixForYawPitchRollConversion);
   }

   public double getPitch()
   {
      tempMatrixForYawPitchRollConversion.set(quaternion);
      return RotationTools.computePitch(tempMatrixForYawPitchRollConversion);
   }

   public double getRoll()
   {
      tempMatrixForYawPitchRollConversion.set(quaternion);
      return RotationTools.computeRoll(tempMatrixForYawPitchRollConversion);
   }

   public Quat4d getQuaternion()
   {
      return quaternion;
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

   public void interpolate(Quat4d quaternion1, Quat4d quaternion2, double alpha)
   {
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
      quaternion.interpolate(quaternion1, quaternion2, alpha);
   }

   public void setOrientationFromOneToTwo(FrameOrientation orientationOne, FrameOrientation orientationTwo)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      this.checkReferenceFrameMatch(orientationOne);

      this.quaternion.conjugate(orientationTwo.quaternion);
      this.quaternion.mul(orientationOne.quaternion);
   }

   public void mul(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      mul(frameOrientation.quaternion);
   }

   public void mul(Quat4d quaternion)
   {
      this.quaternion.mul(quaternion);
   }

   public double dot(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      return dot(frameOrientation.quaternion);
   }

   public double dot(Quat4d quaternion)
   {
      double dot = this.quaternion.x * quaternion.x;
      dot += this.quaternion.y * quaternion.y;
      dot += this.quaternion.z * quaternion.z;
      dot += this.quaternion.w * quaternion.w;
      
      return dot;
   }

   public void negateQuaternion()
   {
      quaternion.negate();
   }

   /**
    * Normalize the quaternion and also limits the described angle magnitude in [-Pi, Pi].
    * The latter prevents some controllers to poop their pants.
    */
   public void normalize()
   {
      if (normSquared() < 1.0e-7)
         setToZero();
      else
      {
         quaternion.normalize();
         if (quaternion.getW() < 0.0)
            negateQuaternion();
      }
   }

   public double getQx()
   {
      return quaternion.x;
   }

   public double getQy()
   {
      return quaternion.y;
   }

   public double getQz()
   {
      return quaternion.z;
   }

   public double getQs()
   {
      return quaternion.w;
   }

   public boolean epsilonEquals(FrameOrientation frameOrientation, double epsilon)
   {
      boolean referenceFramesMatch = referenceFrame == frameOrientation.referenceFrame;
      boolean quaternionsAreEqual = RotationTools.quaternionEpsilonEquals(quaternion, frameOrientation.quaternion, epsilon);

      return referenceFramesMatch && quaternionsAreEqual;
   }

   public boolean epsilonEquals(Quat4d quaternion, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(this.quaternion, quaternion, epsilon);
   }

   public void checkQuaternionIsUnitMagnitude()
   {
      double normSquared = normSquared();
      if (Math.abs(normSquared - 1.0) > 1e-12)
      {
         System.err.println("\nQuaternion " + quaternion + " is not unit magnitude! normSquared = " + normSquared);

         throw new RuntimeException("Quaternion " + quaternion + " is not unit magnitude! normSquared = " + normSquared);
      }
   }

   public double normSquared()
   {
      return quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w;
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
