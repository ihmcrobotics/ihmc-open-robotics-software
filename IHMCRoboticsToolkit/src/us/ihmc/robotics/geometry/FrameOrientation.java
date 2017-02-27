package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameOrientation extends AbstractFrameObject<FrameOrientation, Quaternion>
{
   private final Quaternion quaternion;

   public FrameOrientation(FrameOrientation orientation)
   {
      super(orientation.getReferenceFrame(), new Quaternion(orientation.quaternion));
      quaternion = getGeometryObject();
   }

   public FrameOrientation(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Quaternion());
      quaternion = getGeometryObject();
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
   }

   public FrameOrientation(ReferenceFrame referenceFrame, QuaternionReadOnly quaternion)
   {
      this(referenceFrame);
      this.quaternion.set(quaternion);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this(referenceFrame);
      quaternion.set(qx, qy, qz, qs);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      this(referenceFrame);
      setYawPitchRoll(yaw, pitch, roll);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      this(referenceFrame);
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotation)
   {
      this(referenceFrame);
      quaternion.set(rotation);
   }

   public FrameOrientation(ReferenceFrame referenceFrame, AxisAngleReadOnly orientation)
   {
      this(referenceFrame);
      quaternion.set(orientation);
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame)
   {
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, RandomGeometry.nextQuaternion(random));
      return randomOrientation;
   }

   public static FrameOrientation generateRandomFrameOrientation(Random random, ReferenceFrame referenceFrame, double yawMin, double yawMax, double pitchMin,
                                                                 double pitchMax, double rollMin, double rollMax)
   {
      double yaw = RandomNumbers.nextDouble(random, yawMin, yawMax);
      double pitch = RandomNumbers.nextDouble(random, pitchMin, pitchMax);
      double roll = RandomNumbers.nextDouble(random, rollMin, rollMax);
      FrameOrientation randomOrientation = new FrameOrientation(referenceFrame, yaw, pitch, roll);
      return randomOrientation;
   }

   public void set(AxisAngleReadOnly axisAngle4d)
   {
      quaternion.set(axisAngle4d);
   }

   @Override
   public void set(Quaternion quaternion)
   {
      this.quaternion.set(quaternion);
   }

   public void set(QuaternionReadOnly quaternion)
   {
      this.quaternion.set(quaternion);
   }

   public void set(double qx, double qy, double qz, double qs)
   {
      quaternion.set(qx, qy, qz, qs);
   }

   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      quaternion.set(rotationMatrix);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      quaternion.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   @Override
   public void set(FrameOrientation orientation)
   {
      referenceFrame.checkReferenceFrameMatch(orientation.referenceFrame);
      quaternion.set(orientation.quaternion);
   }

   @Override
   public void setIncludingFrame(ReferenceFrame referenceFrame, Quaternion quaternion)
   {
      this.referenceFrame = referenceFrame;
      set(quaternion);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly quaternion)
   {
      this.referenceFrame = referenceFrame;
      set(quaternion);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double qx, double qy, double qz, double qs)
   {
      this.referenceFrame = referenceFrame;
      set(qx, qy, qz, qs);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
   {
      this.referenceFrame = referenceFrame;
      set(axisAngle);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
   {
      this.referenceFrame = referenceFrame;
      set(rotationMatrix);
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
      setYawPitchRoll(yaw, pitch, roll);
   }

   @Override
   public void setIncludingFrame(FrameOrientation orientation)
   {
      referenceFrame = orientation.referenceFrame;
      quaternion.set(orientation.quaternion);
   }

   @Override
   public boolean containsNaN()
   {
      return quaternion.containsNaN();
   }

   public void getQuaternion(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(quaternion);
   }

   public void getMatrix3d(RotationMatrix matrixToPack)
   {
      matrixToPack.set(quaternion);
   }

   public void getAxisAngle(AxisAngleBasics axisAngleToPack)
   {
      axisAngleToPack.set(quaternion);
   }

   public void getTransform3D(RigidBodyTransform transformToPack)
   {
      transformToPack.setRotation(quaternion);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      quaternion.getYawPitchRoll(yawPitchRollToPack);
   }

   public double[] getYawPitchRoll()
   {
      double[] yawPitchRollToReturn = new double[3];
      getYawPitchRoll(yawPitchRollToReturn);
      return yawPitchRollToReturn;
   }

   public double getYaw()
   {
      return quaternion.getYaw();
   }

   public double getPitch()
   {
      return quaternion.getPitch();
   }

   public double getRoll()
   {
      return quaternion.getRoll();
   }

   public Quaternion getQuaternion()
   {
      return quaternion;
   }

   public Quaternion getQuaternionCopy()
   {
      return new Quaternion(quaternion);
   }

   public RotationMatrix getMatrix3dCopy()
   {
      RotationMatrix ret = new RotationMatrix();
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

   public void interpolate(QuaternionReadOnly quaternion1, QuaternionReadOnly quaternion2, double alpha)
   {
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);
      quaternion.interpolate(quaternion1, quaternion2, alpha);
   }

   public void setOrientationFromOneToTwo(FrameOrientation orientationOne, FrameOrientation orientationTwo)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      this.checkReferenceFrameMatch(orientationOne);

      quaternion.setAndConjugate(orientationTwo.quaternion);
      quaternion.multiply(orientationOne.quaternion);
   }

   public void multiply(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      multiply(frameOrientation.quaternion);
   }

   public void multiply(QuaternionReadOnly quaternion)
   {
      this.quaternion.multiply(quaternion);
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

   public double dot(QuaternionReadOnly quaternion)
   {
      return this.quaternion.dot(quaternion);
   }

   public void negateQuaternion()
   {
      quaternion.negate();
   }

   public void normalize()
   {
      quaternion.normalize();
   }

   /**
    * Normalize the quaternion and also limits the described angle magnitude in [-Pi, Pi]. The
    * latter prevents some controllers to poop their pants.
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
      return quaternion.getS();
   }

   @Override
   public boolean epsilonEquals(FrameOrientation frameOrientation, double epsilon)
   {
      boolean referenceFramesMatch = referenceFrame == frameOrientation.referenceFrame;
      boolean quaternionsAreEqual = RotationTools.quaternionEpsilonEquals(quaternion, frameOrientation.quaternion, epsilon);

      return referenceFramesMatch && quaternionsAreEqual;
   }

   public boolean epsilonEquals(QuaternionReadOnly quaternion, double epsilon)
   {
      return RotationTools.quaternionEpsilonEquals(this.quaternion, quaternion, epsilon);
   }

   public void checkQuaternionIsUnitMagnitude()
   {
      quaternion.checkIfUnitary();
   }

   public double normSquared()
   {
      return quaternion.normSquared();
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
