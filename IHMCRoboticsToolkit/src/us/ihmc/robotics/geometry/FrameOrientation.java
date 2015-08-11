package us.ihmc.robotics.geometry;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import java.util.Random;

// TODO: This should only be orientation. If you want a full transform, you want to use a FramePose.
// So the transform3D should be replaced with a rotation matrix.
// However, people might be using this as a full Pose, so we need to be careful when cleaning it up.
public class FrameOrientation extends ReferenceFrameHolder
{
   private final Vector3d zero = new Vector3d();

   private ReferenceFrame referenceFrame;
   private final RigidBodyTransform transform3D = new RigidBodyTransform();
   private Quat4d tempQuat4d;

   public FrameOrientation(FrameOrientation orientation)
   {
      referenceFrame = orientation.referenceFrame;
      transform3D.set(orientation.transform3D);
   }

   public FrameOrientation(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public FrameOrientation()
   {
      referenceFrame = ReferenceFrame.getWorldFrame();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, RigidBodyTransform transform3D)
   {
      this.referenceFrame = referenceFrame;
      this.transform3D.set(transform3D);
      transform3D.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4d quaternion)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotationAndZeroTranslation(quaternion);
      transform3D.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Quat4f quaternion)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotationAndZeroTranslation(quaternion);
      transform3D.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this.referenceFrame = referenceFrame;

      if (tempQuat4d == null)
         tempQuat4d = new Quat4d();
      tempQuat4d.set(qx, qy, qz, qs);
      transform3D.setRotationAndZeroTranslation(tempQuat4d);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      this.referenceFrame = referenceFrame;
      setYawPitchRoll(yaw, pitch, roll);
      transform3D.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      this.referenceFrame = referenceFrame;
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, Matrix3d rotation)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotationAndZeroTranslation(rotation);
      transform3D.normalize();
   }

   public FrameOrientation(ReferenceFrame referenceFrame, AxisAngle4d orientation)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotationAndZeroTranslation(orientation);
      transform3D.normalize();
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame)
   {
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, RandomTools.generateRandomQuaternion(random));
      return randomOrientation;
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame, double yawMin, double yawMax, double pitchMin,
         double pitchMax, double rollMin, double rollMax)
   {
      double yaw = RandomTools.generateRandomDouble(random, yawMin, yawMax);
      double pitch = RandomTools.generateRandomDouble(random, pitchMin, pitchMax);
      double roll = RandomTools.generateRandomDouble(random, rollMin, rollMax);
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, yaw, pitch, roll);
      return randomOrientation;
   }

   public void set(AxisAngle4d axisAngle4d)
   {
      transform3D.setRotationAndZeroTranslation(axisAngle4d);
      transform3D.normalize();
   }

   public void set(Quat4d quat4d)
   {
      transform3D.setRotationAndZeroTranslation(quat4d);
      transform3D.normalize();
   }

   public void set(Matrix3d rotationMatrix)
   {
      transform3D.setRotationAndZeroTranslation(rotationMatrix);
      transform3D.normalize();
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      if (Double.isNaN(roll))
      {
         throw new RuntimeException("Orientation.setYawPitchRoll(). yaw = " + yaw + ", pitch = " + pitch + ", roll = " + roll);
      }

      transform3D.setEuler(roll, pitch, yaw);
      transform3D.normalize();
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void set(FrameOrientation orientation)
   {
      referenceFrame.checkReferenceFrameMatch(orientation.referenceFrame);
      transform3D.set(orientation.transform3D);
      transform3D.normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Quat4d quat)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotation(quat);
      transform3D.normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, AxisAngle4d axisAngle)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotation(axisAngle);
      transform3D.normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3d rotationMatrix)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setRotationAndZeroTranslation(rotationMatrix);
      transform3D.normalize();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform transform3D)
   {
      this.referenceFrame = referenceFrame;
      this.transform3D.set(transform3D);
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
      transform3D.set(orientation.transform3D);
      transform3D.normalize();
   }

   // TODO Find a better. I chose setToZero() as in FrameTuple.
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      transform3D.setIdentity();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void getQuaternion(Quat4d quat4d)
   {
      RotationFunctions.setQuaternionBasedOnTransform(quat4d, transform3D);
   }

   public void getMatrix3d(Matrix3d matrixToPack)
   {
      transform3D.get(matrixToPack);
   }

   public void getAxisAngle(AxisAngle4d axisAngleToPack)
   {
      transform3D.getRotation(axisAngleToPack);
   }

   public void getTransform3D(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform3D);
   }

   private final Matrix3d rotationMatrix = new Matrix3d();

   public void getYawPitchRoll(double[] yawPitchRoll)
   {
      // This seems to work much better than going to quaternions first, especially when yaw is large...
      transform3D.get(rotationMatrix);
      yawPitchRoll[0] = Math.atan2(rotationMatrix.m10, rotationMatrix.m00);

      if (Math.abs(rotationMatrix.m20) < 1.0 - 1e-10)
         yawPitchRoll[1] = Math.asin(-rotationMatrix.m20);
      else
         yawPitchRoll[1] = -Math.signum(rotationMatrix.m20) * Math.PI / 2.0;

      yawPitchRoll[2] = Math.atan2(rotationMatrix.m21, rotationMatrix.m22);

      if (Double.isNaN(yawPitchRoll[0]) || Double.isNaN(yawPitchRoll[1]) || Double.isNaN(yawPitchRoll[2]))
      {
         throw new RuntimeException("yaw, pitch, or roll are NaN! transform3D = " + transform3D);
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
      transform3D.get(rotationMatrix);
      return RotationFunctions.getYaw(rotationMatrix);
   }

   public double getPitch()
   {
      transform3D.get(rotationMatrix);
      return RotationFunctions.getPitch(rotationMatrix);
   }

   public double getRoll()
   {
      transform3D.get(rotationMatrix);
      return RotationFunctions.getRoll(rotationMatrix);
   }

   public Quat4d getQuaternionCopy()
   {
      Quat4d quaternionToReturn = new Quat4d();

      RotationFunctions.setQuaternionBasedOnTransform(quaternionToReturn, transform3D);

      return quaternionToReturn;
   }

   public Matrix3d getMatrix3dCopy()
   {
      Matrix3d ret = new Matrix3d();
      transform3D.getRotation(ret);

      return ret;
   }

   public void getFrameOrientation2dIncludingFrame(FrameOrientation2d frameOrientation2dToPack)
   {
      frameOrientation2dToPack.setIncludingFrame(referenceFrame, getYaw());
   }

   public void applyTransform(RigidBodyTransform transform3D)
   {
      this.transform3D.multiply(transform3D, this.transform3D);
      this.transform3D.setTranslation(zero);
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
         transform3D.multiply(referenceTf, transform3D);
      }

      if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
      {
         transform3D.multiply(desiredTf, transform3D);
      }

      transform3D.setTranslation(zero);
      referenceFrame = desiredFrame;
      transform3D.normalize();
   }

   public void interpolate(FrameOrientation orientationOne, FrameOrientation orientationTwo, double alpha)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      if (tempQuat4d == null)
         tempQuat4d = new Quat4d();
      if (orientationOne.tempQuat4d == null)
         orientationOne.tempQuat4d = new Quat4d();
      if (orientationTwo.tempQuat4d == null)
         orientationTwo.tempQuat4d = new Quat4d();

      RotationFunctions.setQuaternionBasedOnTransform(orientationOne.tempQuat4d, orientationOne.transform3D);
      RotationFunctions.setQuaternionBasedOnTransform(orientationTwo.tempQuat4d, orientationTwo.transform3D);

      tempQuat4d.interpolate(orientationOne.tempQuat4d, orientationTwo.tempQuat4d, alpha);

      double normSquared = (tempQuat4d.x * tempQuat4d.x + tempQuat4d.y * tempQuat4d.y + tempQuat4d.z * tempQuat4d.z + tempQuat4d.w * tempQuat4d.w);

      MathTools.checkIfEqual(1.0, normSquared, 1e-12);

      this.set(tempQuat4d);

      referenceFrame = orientationOne.getReferenceFrame();
      transform3D.normalize();
   }

   
   public void setOrientationFromOneToTwo(FrameOrientation orientationOne, FrameOrientation orientationTwo)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      this.checkReferenceFrameMatch(orientationOne);
      
      this.transform3D.set(orientationTwo.transform3D);
      this.transform3D.invert();
      this.transform3D.multiply(orientationOne.transform3D);
   }
   
   
   public boolean epsilonEquals(FrameOrientation frameOrientation, double epsilon)
   {
      boolean referenceFramesMatch = referenceFrame == frameOrientation.referenceFrame;
      boolean transformsAreEqual = transform3D.epsilonEquals(frameOrientation.transform3D, epsilon);

      return referenceFramesMatch && transformsAreEqual;
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
      Quat4d quaternion = getQuaternionCopy();

      return "quaternion: " + quaternion + "-" + referenceFrame.getName();
   }
}
