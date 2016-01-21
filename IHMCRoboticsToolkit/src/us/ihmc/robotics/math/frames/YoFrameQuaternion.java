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
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameQuaternion extends ReferenceFrameHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable qx, qy, qz, qs;
   private final Quat4d quaternion = new Quat4d();
   private final Quat4d tempQuaternion2 = new Quat4d();
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

   public void set(Quat4d quat)
   {
      quaternion.set(quat);
      getYoValuesFromQuat4d();
   }

   public void set(Matrix3d matrix)
   {
      RotationTools.setQuaternionBasedOnMatrix3d(quaternion, matrix);
      getYoValuesFromQuat4d();
   }

   public void set(AxisAngle4d axisAngle)
   {
      quaternion.set(axisAngle);
      getYoValuesFromQuat4d();
   }

   public void set(double[] yawPitchRoll)
   {
      RotationTools.setQuaternionBasedOnYawPitchRoll(quaternion, yawPitchRoll);
      getYoValuesFromQuat4d();
   }

   public void set(double yaw, double pitch, double roll)
   {
      RotationTools.setQuaternionBasedOnYawPitchRoll(quaternion, yaw, pitch, roll);
      getYoValuesFromQuat4d();
   }

   public void set(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      frameOrientation.getQuaternion(quaternion);
      getYoValuesFromQuat4d();
   }

   public void set(YoFrameQuaternion yoFrameQuaternion)
   {
      checkReferenceFrameMatch(yoFrameQuaternion);
      yoFrameQuaternion.get(quaternion);
      getYoValuesFromQuat4d();
   }

   public void get(Quat4d quat)
   {
      putYoValuesIntoQuat4d();
      quat.set(quaternion);
   }

   //   public void get(Quat4f quat)
   //   {
   //      putYoValuesIntoQuat4d();
   //      quat.set(quaternion);
   //   }

   public void get(Matrix3d matrix)
   {
      putYoValuesIntoQuat4d();
      matrix.set(quaternion);
   }

   public void get(Matrix3f matrix)
   {
      putYoValuesIntoQuat4d();
      matrix.set(quaternion);
   }

   public void get(AxisAngle4d axisAngle)
   {
      putYoValuesIntoQuat4d();
      axisAngle.set(quaternion);
   }

   public void getYawPitchRoll(double[] yawPitchRoll)
   {
      putYoValuesIntoQuat4d();
      RotationTools.setYawPitchRollBasedOnQuaternion(yawPitchRoll, quaternion);
   }

   public void getFrameOrientationIncludingFrame(FrameOrientation frameOrientation)
   {
      putYoValuesIntoQuat4d();
      frameOrientation.setIncludingFrame(getReferenceFrame(), quaternion);
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
      checkReferenceFrameMatch(yoFrameQuaternion1);
      checkReferenceFrameMatch(yoFrameQuaternion2);

      yoFrameQuaternion1.putYoValuesIntoQuat4d();
      yoFrameQuaternion2.putYoValuesIntoQuat4d();

      interpolate(yoFrameQuaternion1.quaternion, yoFrameQuaternion2.quaternion, alpha);
   }

   public void interpolate(Quat4d quaternion1, Quat4d quaternion2, double alpha)
   {
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      quaternion.interpolate(quaternion1, quaternion2, alpha); 
      checkQuaternionIsUnitMagnitude(quaternion);
      getYoValuesFromQuat4d();
   }
   public void checkQuaternionIsUnitMagnitude()
   {
      putYoValuesIntoQuat4d();
      checkQuaternionIsUnitMagnitude(this.quaternion);
   }

   private static void checkQuaternionIsUnitMagnitude(Quat4d quaternion)
   {
      double normSquared = (quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w);
      if (Math.abs(normSquared - 1.0) > 1e-12)
      {
         System.err.println("\nQuaternion " + quaternion + " is not unit magnitude! normSquared = " + normSquared);

         throw new RuntimeException("Quaternion " + quaternion + " is not unit magnitude! normSquared = " + normSquared);
      }
   }

   /**
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the orientation represented by the FrameOrientation.
    * @param quat4d
    */
   public void mul(Quat4d quat4d)
   {
      putYoValuesIntoQuat4d();
      quaternion.mul(quat4d);
      getYoValuesFromQuat4d();
   }

   /**
    * Method used to concatenate the orientation represented by this YoFrameQuaternion and the orientation represented by the FrameOrientation.
    * @param frameOrientation
    */
   public void mul(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation.getReferenceFrame());
      frameOrientation.getQuaternion(tempQuaternion2);
      mul(tempQuaternion2);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   private void getYoValuesFromQuat4d()
   {
      qx.set(quaternion.getX());
      qy.set(quaternion.getY());
      qz.set(quaternion.getZ());
      qs.set(quaternion.getW());
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

   private void putYoValuesIntoQuat4d()
   {
      quaternion.set(qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue(), qs.getDoubleValue());
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
