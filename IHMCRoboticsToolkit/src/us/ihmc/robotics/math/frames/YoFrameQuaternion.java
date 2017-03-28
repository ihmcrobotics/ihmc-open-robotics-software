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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly since they contain YoVariables.
public class YoFrameQuaternion extends AbstractReferenceFrameHolder implements Clearable
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable qx, qy, qz, qs;
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

      this.qx = new DoubleYoVariable(createQxName(namePrefix, nameSuffix), registry);
      this.qy = new DoubleYoVariable(createQyName(namePrefix, nameSuffix), registry);
      this.qz = new DoubleYoVariable(createQzName(namePrefix, nameSuffix), registry);
      this.qs = new DoubleYoVariable(createQsName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;

      qs.set(1.0);
   }

   public YoFrameQuaternion(DoubleYoVariable qx, DoubleYoVariable qy, DoubleYoVariable qz, DoubleYoVariable qs, ReferenceFrame referenceFrame)
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

   public DoubleYoVariable getYoQx()
   {
      return qx;
   }

   public DoubleYoVariable getYoQy()
   {
      return qy;
   }

   public DoubleYoVariable getYoQz()
   {
      return qz;
   }

   public DoubleYoVariable getYoQs()
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
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the
    * orientation represented by the FrameOrientation.
    * 
    * @param quaternion
    */
   public void multiply(Quaternion quaternion)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.multiply(quaternion);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the
    * orientation represented by the FrameOrientation.
    * 
    * @param frameOrientation
    */
   public void multiply(FrameOrientation frameOrientation)
   {
      putYoValuesIntoFrameOrientation();
      this.frameOrientation.multiply(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   public void conjugate()
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.conjugate();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Compute the dot product between this quaternion and the orhter quaternion: this . other = qx *
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
