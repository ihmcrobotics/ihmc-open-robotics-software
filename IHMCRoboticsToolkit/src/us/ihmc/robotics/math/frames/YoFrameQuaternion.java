package us.ihmc.robotics.math.frames;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQsName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQxName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQyName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQzName;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly since they contain YoVariables.
public class YoFrameQuaternion extends AbstractReferenceFrameHolder implements Clearable
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble qx, qy, qz, qs;
   private final FrameOrientation frameOrientation = new FrameOrientation();
   /**
    * Never use this reference frame directly, use {@link #getReferenceFrame()} instead so the
    * multiple frames version of this {@link YoFrameQuaternion} will work properly.
    */
   private final ReferenceFrame referenceFrame;

   public YoFrameQuaternion(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameQuaternion(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      this.qx = new YoDouble(createQxName(namePrefix, nameSuffix), registry);
      this.qy = new YoDouble(createQyName(namePrefix, nameSuffix), registry);
      this.qz = new YoDouble(createQzName(namePrefix, nameSuffix), registry);
      this.qs = new YoDouble(createQsName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;

      qs.set(1.0);
   }

   public YoFrameQuaternion(YoDouble qx, YoDouble qy, YoDouble qz, YoDouble qs, ReferenceFrame referenceFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(qx.getName(), qy.getName(), qz.getName(), qs.getName());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(qx.getName(), qy.getName(), qz.getName(), qs.getName());

      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qs = qs;
      this.referenceFrame = referenceFrame;
   }

   public final FrameOrientation getFrameOrientation()
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation;
   }

   public void set(QuaternionReadOnly quaternion)
   {
      frameOrientation.set(quaternion);
      getYoValuesFromFrameOrientation();
   }

   public void set(RotationMatrixReadOnly matrix)
   {
      frameOrientation.set(matrix);
      getYoValuesFromFrameOrientation();
   }

   public void set(AxisAngleReadOnly axisAngle)
   {
      frameOrientation.set(axisAngle);
      getYoValuesFromFrameOrientation();
   }

   public void set(double[] yawPitchRoll)
   {
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      getYoValuesFromFrameOrientation();
   }

   public void set(double yaw, double pitch, double roll)
   {
      frameOrientation.setYawPitchRoll(yaw, pitch, roll);
      getYoValuesFromFrameOrientation();
   }

   public void set(double qx, double qy, double qz, double qs)
   {
      this.qx.set(qx);
      this.qy.set(qy);
      this.qz.set(qz);
      this.qs.set(qs);
   }

   public void set(FrameOrientation frameOrientation)
   {
      set(frameOrientation, true);
   }

   public void set(FrameOrientation frameOrientation, boolean notifyListeners)
   {
      checkReferenceFrameMatch(frameOrientation);
      this.frameOrientation.setIncludingFrame(frameOrientation);
      getYoValuesFromFrameOrientation(notifyListeners);
   }

   public void setAndMatchFrame(FrameOrientation frameOrientation)
   {
      setAndMatchFrame(frameOrientation, true);
   }

   public void setAndMatchFrame(FrameOrientation frameOrientation, boolean notifyListeners)
   {
      this.frameOrientation.setIncludingFrame(frameOrientation);
      this.frameOrientation.changeFrame(getReferenceFrame());
      getYoValuesFromFrameOrientation(notifyListeners);
   }

   public void set(YoFrameQuaternion yoFrameQuaternion)
   {
      checkReferenceFrameMatch(yoFrameQuaternion);
      yoFrameQuaternion.getFrameOrientationIncludingFrame(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Sets the orientation of this to the origin of the passed in ReferenceFrame.
    *
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame, boolean notifyListeners)
   {
      frameOrientation.setToZero(referenceFrame);
      frameOrientation.changeFrame(getReferenceFrame());
      getYoValuesFromFrameOrientation(notifyListeners);
   }

   /**
    * Sets the orientation of this to the origin of the passed in ReferenceFrame.
    *
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setFromReferenceFrame(referenceFrame, true);
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
    * @param rotation vector the rotation vector used to set this {@code YoFrameQuaternion}. Not
    *           modified.
    */
   public void setRotationVector(Vector3DReadOnly rotationVector)
   {
      frameOrientation.setToZero(getReferenceFrame());
      frameOrientation.setRotationVector(rotationVector);
      getYoValuesFromFrameOrientation();
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
    * @param rotation vector the rotation vector used to set this {@code YoFrameQuaternion}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void setRotationVector(FrameVector rotationVector)
   {
      frameOrientation.setToZero(getReferenceFrame());
      frameOrientation.setRotationVector(rotationVector);
      getYoValuesFromFrameOrientation();
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
    * @param rotation vector the rotation vector used to set this {@code YoFrameQuaternion}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void setRotationVector(YoFrameVector rotationVector)
   {
      frameOrientation.setToZero(getReferenceFrame());
      frameOrientation.setRotationVector(rotationVector.getFrameTuple());
      getYoValuesFromFrameOrientation();
   }

   public void get(QuaternionBasics quaternionToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getQuaternion(quaternionToPack);
   }

   public void get(RotationMatrix matrixToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getMatrix3d(matrixToPack);
   }

   public void get(AxisAngleBasics axisAngleToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getAxisAngle(axisAngleToPack);
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getYawPitchRoll(yawPitchRollToPack);
   }

   public void getFrameOrientation(FrameOrientation frameOrientationToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientationToPack.set(this.frameOrientation);
   }

   public void getFrameOrientationIncludingFrame(FrameOrientation frameOrientationToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientationToPack.setIncludingFrame(frameOrientation);
   }

   public Quaternion getQuaternionCopy()
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation.getQuaternionCopy();
   }

   public FrameOrientation getFrameOrientationCopy()
   {
      putYoValuesIntoFrameOrientation();
      return new FrameOrientation(frameOrientation);
   }

   /**
    * Computes and packs the orientation described by this {@code YoFrameQuaternion} as a rotation
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
   public void getRotationVector(FrameVector frameRotationVectorToPack)
   {
      getFrameOrientation().getRotationVector(frameRotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this {@code YoFrameQuaternion} as a rotation
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
   public void getRotationVectorIncludingFrame(FrameVector frameRotationVectorToPack)
   {
      getFrameOrientation().getRotationVectorIncludingFrame(frameRotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by this {@code YoFrameQuaternion} as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param yoFrameRotationVectorToPack the vector in which the rotation vector and the reference
    *           frame it is expressed in are stored. Modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void getRotationVector(YoFrameVector yoFrameRotationVectorToPack)
   {
      yoFrameRotationVectorToPack.setAsRotationVector(getFrameOrientation());
   }

   public YoDouble getYoQx()
   {
      return qx;
   }

   public YoDouble getYoQy()
   {
      return qy;
   }

   public YoDouble getYoQz()
   {
      return qz;
   }

   public YoDouble getYoQs()
   {
      return qs;
   }

   public double getQx()
   {
      return qx.getDoubleValue();
   }

   public double getQy()
   {
      return qy.getDoubleValue();
   }

   public double getQz()
   {
      return qz.getDoubleValue();
   }

   public double getQs()
   {
      return qs.getDoubleValue();
   }

   public void interpolate(YoFrameQuaternion yoFrameQuaternion1, YoFrameQuaternion yoFrameQuaternion2, double alpha)
   {
      yoFrameQuaternion1.putYoValuesIntoFrameOrientation();
      yoFrameQuaternion2.putYoValuesIntoFrameOrientation();

      interpolate(yoFrameQuaternion1.frameOrientation, yoFrameQuaternion2.frameOrientation, alpha);
   }

   public void interpolate(FrameOrientation frameOrientation1, FrameOrientation frameOrientation2, double alpha)
   {
      checkReferenceFrameMatch(frameOrientation1);
      checkReferenceFrameMatch(frameOrientation2);

      frameOrientation.interpolate(frameOrientation1, frameOrientation2, alpha);
      frameOrientation.checkQuaternionIsUnitMagnitude();
      getYoValuesFromFrameOrientation();
   }

   public void interpolate(QuaternionReadOnly quaternion1, QuaternionReadOnly quaternion2, double alpha)
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      frameOrientation.interpolate(quaternion1, quaternion2, alpha);
      frameOrientation.checkQuaternionIsUnitMagnitude();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Multiplies this quaternion by {@code quaternion}.
    * <p>
    * this = this * quaternion
    * </p>
    *
    * @param quaternion the other quaternion to multiply this. Not modified.
    */
   public void multiply(Quaternion quaternion)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.multiply(quaternion);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Multiplies this quaternion by {@code frameOrientation}.
    * <p>
    * this = this * frameOrientation.quaternion
    * </p>
    *
    * @param frameOrientation the frame orientation to multiply this. Not modified.
    */
   public void multiply(FrameOrientation frameOrientation)
   {
      putYoValuesIntoFrameOrientation();
      this.frameOrientation.multiply(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Multiplies this quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the quaternion to multiply this. Not modified.
    */
   public void multiply(YoFrameQuaternion other)
   {
      putYoValuesIntoFrameOrientation();
      this.frameOrientation.multiply(other.getFrameOrientation());
      getYoValuesFromFrameOrientation();
   }

   public void conjugate()
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.conjugate();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Compute the dot product between this quaternion and the other quaternion: this . other = qx *
    * other.qx + qy * other.qy + qz * other.qz + qs * other.qs.
    * 
    * @param other
    * @return
    */
   public double dot(YoFrameQuaternion other)
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation.dot(other.frameOrientation);
   }

   public void negate()
   {
      qx.set(-qx.getDoubleValue());
      qy.set(-qy.getDoubleValue());
      qz.set(-qz.getDoubleValue());
      qs.set(-qs.getDoubleValue());
   }

   public void checkQuaternionIsUnitMagnitude()
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.checkQuaternionIsUnitMagnitude();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   private void getYoValuesFromFrameOrientation()
   {
      getYoValuesFromFrameOrientation(true);
   }

   private void getYoValuesFromFrameOrientation(boolean notifyListeners)
   {
      qx.set(frameOrientation.getQx(), notifyListeners);
      qy.set(frameOrientation.getQy(), notifyListeners);
      qz.set(frameOrientation.getQz(), notifyListeners);
      qs.set(frameOrientation.getQs(), notifyListeners);
   }

   @Override
   public void setToNaN()
   {
      frameOrientation.setToNaN();
      getYoValuesFromFrameOrientation();
   }

   @Override
   public void setToZero()
   {
      frameOrientation.setToZero(getReferenceFrame());
      getYoValuesFromFrameOrientation();
   }

   @Override
   public boolean containsNaN()
   {
      return qx.isNaN() || qy.isNaN() || qz.isNaN() || qs.isNaN();
   }

   private void putYoValuesIntoFrameOrientation()
   {
      frameOrientation.setIncludingFrame(getReferenceFrame(), qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue(), qs.getDoubleValue());
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      qx.addVariableChangedListener(variableChangedListener);
      qy.addVariableChangedListener(variableChangedListener);
      qz.addVariableChangedListener(variableChangedListener);
      qs.addVariableChangedListener(variableChangedListener);
   }

   /**
    * toString
    *
    * String representation of a FrameVector (qx, qy, qz, qs)-reference frame name
    *
    * @return String
    */
   @Override
   public String toString()
   {
      return "(" + qx.getDoubleValue() + ", " + qy.getDoubleValue() + ", " + qz.getDoubleValue() + ", " + qs.getDoubleValue() + ")-" + getReferenceFrame();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   public boolean epsilonEquals(YoFrameQuaternion other, double epsilon)
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation.epsilonEquals(other.frameOrientation, epsilon);
   }
}
