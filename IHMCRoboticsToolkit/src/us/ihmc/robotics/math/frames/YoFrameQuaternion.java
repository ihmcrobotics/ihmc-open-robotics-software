package us.ihmc.robotics.math.frames;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQsName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQxName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQyName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQzName;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4d;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public class YoFrameQuaternion extends ReferenceFrameHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable qx, qy, qz, qs;
   private final FrameOrientation frameOrientation = new FrameOrientation();
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

   public void set(Quat4d quaternion)
   {
      frameOrientation.set(quaternion);
      getYoValuesFromFrameOrientation();
   }

   public void set(Matrix3d matrix)
   {
      frameOrientation.set(matrix);
      getYoValuesFromFrameOrientation();
   }

   public void set(AxisAngle4d axisAngle)
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

   public void set(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      this.frameOrientation.setIncludingFrame(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   public void set(YoFrameQuaternion yoFrameQuaternion)
   {
      checkReferenceFrameMatch(yoFrameQuaternion);
      yoFrameQuaternion.getFrameOrientationIncludingFrame(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   public void get(Quat4d quaternionToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getQuaternion(quaternionToPack);
   }

   public void get(Matrix3d matrixToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getMatrix3d(matrixToPack);
   }

   public void get(Matrix3f matrixToPack)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.getMatrix3f(matrixToPack);
   }

   public void get(AxisAngle4d axisAngleToPack)
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

   public void interpolate(Quat4d quaternion1, Quat4d quaternion2, double alpha)
   {
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      frameOrientation.interpolate(quaternion1, quaternion2, alpha); 
      frameOrientation.checkQuaternionIsUnitMagnitude();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the orientation represented by the FrameOrientation.
    * @param quaternion
    */
   public void mul(Quat4d quaternion)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.mul(quaternion);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the orientation represented by the FrameOrientation.
    * @param frameOrientation
    */
   public void mul(FrameOrientation frameOrientation)
   {
      putYoValuesIntoFrameOrientation();
      this.frameOrientation.mul(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Compute the dot product between this quaternion and the orhter quaternion: this . other = qx * other.qx + qy * other.qy + qz * other.qz + qs * other.qs.
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
      qx.set(- qx.getDoubleValue());
      qy.set(- qy.getDoubleValue());
      qz.set(- qz.getDoubleValue());
      qs.set(- qs.getDoubleValue());
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
      qx.set(frameOrientation.getQx());
      qy.set(frameOrientation.getQy());
      qz.set(frameOrientation.getQz());
      qs.set(frameOrientation.getQs());
   }

   public void setToNaN()
   {
      qx.set(Double.NaN);
      qy.set(Double.NaN);
      qz.set(Double.NaN);
      qs.set(Double.NaN);
   }

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
}
