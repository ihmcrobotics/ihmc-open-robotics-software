package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.random.RandomGeometry;

public class FrameOrientation extends FrameGeometryObject<FrameOrientation, Quaternion>
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

   public void appendYawRotation(double yawToAppend)
   {
      quaternion.appendYawRotation(yawToAppend);
   }

   public void appendPitchRotation(double pitchToAppend)
   {
      quaternion.appendPitchRotation(pitchToAppend);
   }

   public void appendRollRotation(double rollToAppend)
   {
      quaternion.appendRollRotation(rollToAppend);
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
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      quaternion.interpolate(orientationOne.quaternion, orientationTwo.quaternion, alpha);
      referenceFrame = orientationOne.getReferenceFrame();
   }

   public void interpolate(QuaternionReadOnly quaternion1, QuaternionReadOnly quaternion2, double alpha)
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      quaternion.interpolate(quaternion1, quaternion2, alpha);
   }

   public void setOrientationFromOneToTwo(FrameOrientation orientationOne, FrameOrientation orientationTwo)
   {
      orientationOne.checkReferenceFrameMatch(orientationTwo);
      this.checkReferenceFrameMatch(orientationOne);

      quaternion.setAndConjugate(orientationTwo.quaternion);
      quaternion.multiply(orientationOne.quaternion);
   }

   /**
    * Sets this quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this {@code FrameOrientation}. Not
    *           modified.
    */
   public void setRotationVector(Vector3DReadOnly rotationVector)
   {
      quaternion.set(rotationVector);
   }

   /**
    * Sets this quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this {@code FrameOrientation}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void setRotationVector(FrameVector3D rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      quaternion.set(rotationVector.getVector());
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

   public void preMultiply(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      preMultiply(frameOrientation.quaternion);
   }

   public void preMultiply(QuaternionReadOnly quaternion)
   {
      this.quaternion.preMultiply(quaternion);
   }

   /**
    * Sets this {@code FrameOrientation} to the difference of {@code q1} and {@code q2}.
    * <p>
    * this.quaternion = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first quaternion in the difference. Not modified.
    * @param q2 the second quaternion in the difference. Not modified.
    */
   public void difference(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      quaternion.difference(q1, q2);
   }

   /**
    * Sets this {@code FrameOrientation} to the difference of {@code orientation1} and
    * {@code orientation2}.
    * <p>
    * this.quaternion = orientation1.quaternion<sup>-1</sup> * orientation2.quaternion
    * </p>
    *
    * @param orientation1 the first {@code FrameOrientation} in the difference. Not modified.
    * @param orientation2 the second {@code FrameOrientation} in the difference. Not modified.
    * @throws ReferenceFrameMismatchException if the reference frame of any of the two arguments is
    *            different from the reference frame of {@code this}.
    */
   public void difference(FrameOrientation orientation1, FrameOrientation orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      checkReferenceFrameMatch(orientation2);
      difference(orientation1.getQuaternion(), orientation2.getQuaternion());
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
      quaternion.normalizeAndLimitToPi();
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

   /**
    * Computes and packs the orientation described by this {@code FrameOrientation} as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param frameRotationVectorToPack the vector in which the rotation vector and the reference
    *           frame it is expressed in are stored. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void getRotationVector(FrameVector3D frameRotationVectorToPack)
   {
      checkReferenceFrameMatch(frameRotationVectorToPack);
      quaternion.get(frameRotationVectorToPack.getVector());
   }
   
   /**
    * Computes and packs the orientation described by this {@code FrameOrientation} as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   public void getRotationVector(Vector3D rotationVectorToPack)
   {
      quaternion.get(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this {@code FrameOrientation} as a rotation
    * vector including the reference frame it is expressed in.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param frameRotationVectorToPack the vector in which the rotation vector and the reference
    *           frame it is expressed in are stored. Modified.
    */
   public void getRotationVectorIncludingFrame(FrameVector3D frameRotationVectorToPack)
   {
      frameRotationVectorToPack.setToZero(getReferenceFrame());
      quaternion.get(frameRotationVectorToPack.getVector());
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
