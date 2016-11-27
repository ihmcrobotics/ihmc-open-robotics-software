package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameOrientation extends AbstractFrameObject<FrameOrientation, TransformableQuat4d>
{
   private final TransformableQuat4d quaternion;
   private final Matrix3d tempMatrixForYawPitchRollConversion = new Matrix3d();

   public FrameOrientation(FrameOrientation orientation)
   {
      super(orientation.getReferenceFrame(), new TransformableQuat4d(orientation.quaternion));
      this.quaternion = this.getGeometryObject();
   }

   public FrameOrientation(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new TransformableQuat4d());
      this.quaternion = this.getGeometryObject();
      setToZero(referenceFrame);
   }

   public FrameOrientation()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameOrientation(ReferenceFrame referenceFrame, RigidBodyTransform transform3D)
   {
      this(referenceFrame);
      transform3D.getRotation(quaternion);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4d quaternion)
   {
      this(referenceFrame);
      this.quaternion.set(quaternion);
      this.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4f quaternion)
   {
      this(referenceFrame);
      this.quaternion.set(quaternion);
      this.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this(referenceFrame);

      quaternion.set(qx, qy, qz, qs);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      this(referenceFrame);
      setYawPitchRoll(yaw, pitch, roll);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      this(referenceFrame);
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Matrix3d rotation)
   {
      this(referenceFrame);
      RotationTools.convertMatrixToQuaternion(rotation, quaternion);
      normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, AxisAngle4d orientation)
   {
      this(referenceFrame);
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
      quaternion.setOrientation(axisAngle4d);
   }

   public void set(Quat4d quaternion)
   {
      this.quaternion.set(quaternion);
      normalize();
   }

   public void set(Quat4f quaternion)
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

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4f quaternion)
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
      transform3D.getRotation(quaternion);
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

//   // TODO Find a better. I chose setToZero() as in FrameTuple.
//   public void setToZero(ReferenceFrame referenceFrame)
//   {
//      this.referenceFrame = referenceFrame;
//      setToZero();
//   }

//   // TODO Find a better. I chose setToZero() as in FrameTuple.
//   public void setToZero()
//   {
//      quaternion.set(0.0, 0.0, 0.0, 1.0);
//   }

//   public void setToNaN(ReferenceFrame referenceFrame)
//   {
//      this.referenceFrame = referenceFrame;
//      setToNaN();
//   }

//   public void setToNaN()
//   {
//      quaternion.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
//   }

   public boolean containsNaN()
   {
      return Double.isNaN(quaternion.getX()) || Double.isNaN(quaternion.getY()) || Double.isNaN(quaternion.getZ()) || Double.isNaN(quaternion.getW());
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
      RotationTools.convertQuaternionToYawPitchRoll(quaternion, yawPitchRollToPack);
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

   public void conjugate()
   {
      quaternion.conjugate();
   }

   public double dot(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      return dot(frameOrientation.quaternion);
   }

   public double dot(Quat4d quaternion)
   {
      double dot = this.quaternion.getX() * quaternion.getX();
      dot += this.quaternion.getY() * quaternion.getY();
      dot += this.quaternion.getZ() * quaternion.getZ();
      dot += this.quaternion.getW() * quaternion.getW();
      
      return dot;
   }

   public void negateQuaternion()
   {
      quaternion.negate();
   }

   public void normalize()
   {
      if (containsNaN())
         return;
      quaternion.normalize();
   }
   
   /**
    * Normalize the quaternion and also limits the described angle magnitude in [-Pi, Pi].
    * The latter prevents some controllers to poop their pants.
    */
   public void normalizeAndLimitToPiMinusPi()
   {
      quaternion.normalizeAndLimitToPiMinusPi();
   }

   public double getQx()
   {
      return quaternion.getX();
   }

   public double getQy()
   {
      return quaternion.getY();
   }

   public double getQz()
   {
      return quaternion.getZ();
   }

   public double getQs()
   {
      return quaternion.getW();
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
      return quaternion.getX() * quaternion.getX() + quaternion.getY() * quaternion.getY() + quaternion.getZ() * quaternion.getZ() + quaternion.getW() * quaternion.getW();
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
