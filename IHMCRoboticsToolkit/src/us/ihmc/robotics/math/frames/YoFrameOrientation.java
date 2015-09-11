package us.ihmc.robotics.math.frames;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;


//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameOrientation extends ReferenceFrameHolder
{
   private final DoubleYoVariable yaw, pitch, roll; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   private final double[] tempYawPitchRoll = new double[3];
   private final FrameOrientation tempFrameOrientation = new FrameOrientation();

   public YoFrameOrientation(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameOrientation(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      yaw = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "yaw", nameSuffix), registry);
      pitch = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "pitch", nameSuffix), registry);
      roll = new DoubleYoVariable(YoFrameVariableNameTools.createName(namePrefix, "roll", nameSuffix), registry);

      this.referenceFrame = referenceFrame;

      // frameVector = new FrameVector(frame);
   }

   public YoFrameOrientation(DoubleYoVariable yaw, DoubleYoVariable pitch, DoubleYoVariable roll, ReferenceFrame referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.referenceFrame = referenceFrame;
   }

   public void setEulerAngles(Vector3d eulerAngles)
   {
      setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      setYawPitchRoll(yaw, pitch, roll, true);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, boolean notifyListeners)
   {
      this.yaw.set(yaw, notifyListeners);
      this.pitch.set(pitch, notifyListeners);
      this.roll.set(roll, notifyListeners);
   }

   public void set(Matrix3d rotation)
   {
      tempFrameOrientation.set(rotation);
      set(tempFrameOrientation);
   }

   public void set(Quat4d quaternion)
   {
      tempFrameOrientation.setIncludingFrame(getReferenceFrame(), quaternion);
      set(tempFrameOrientation);
   }

   public void set(RigidBodyTransform transform3D)
   {
      tempFrameOrientation.setIncludingFrame(getReferenceFrame(), transform3D);
      set(tempFrameOrientation);
   }

   public void set(FrameOrientation orientation)
   {
      set(orientation, true);
   }

   public void set(FrameOrientation orientation, boolean notifyListeners)
   {
      orientation.checkReferenceFrameMatch(getReferenceFrame());
      orientation.getYawPitchRoll(tempYawPitchRoll);
      yaw.set(tempYawPitchRoll[0], notifyListeners);
      pitch.set(tempYawPitchRoll[1], notifyListeners);
      roll.set(tempYawPitchRoll[2], notifyListeners);
   }

   public void set(YoFrameOrientation orientation)
   {
      orientation.checkReferenceFrameMatch(getReferenceFrame());
      yaw.set(orientation.yaw.getDoubleValue());
      pitch.set(orientation.pitch.getDoubleValue());
      roll.set(orientation.roll.getDoubleValue());
   }

   public void set(YoFrameQuaternion orientation)
   {
      orientation.checkReferenceFrameMatch(getReferenceFrame());
      orientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      set(tempFrameOrientation);
   }

   public void setAndMatchFrame(FrameOrientation orientation)
   {
      setAndMatchFrame(orientation, true);
   }

   public void setAndMatchFrame(FrameOrientation orientation, boolean notifyListeners)
   {
      tempFrameOrientation.setIncludingFrame(orientation);
      tempFrameOrientation.changeFrame(getReferenceFrame());
      tempFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      yaw.set(tempYawPitchRoll[0], notifyListeners);
      pitch.set(tempYawPitchRoll[1], notifyListeners);
      roll.set(tempYawPitchRoll[2], notifyListeners);
   }

   public void setToNaN()
   {
      yaw.set(Double.NaN);
      pitch.set(Double.NaN);
      roll.set(Double.NaN);
   }

   public double[] getYawPitchRoll()
   {
      return new double[] { yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue() };
   }

   public void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      yawPitchRollToPack[0] = yaw.getDoubleValue();
      yawPitchRollToPack[1] = pitch.getDoubleValue();
      yawPitchRollToPack[2] = roll.getDoubleValue();
   }

   public DoubleYoVariable getYaw()
   {
      return yaw;
   }

   public DoubleYoVariable getPitch()
   {
      return pitch;
   }

   public DoubleYoVariable getRoll()
   {
      return roll;
   }

   public void getEulerAngles(Vector3d eulerAnglesToPack)
   {
      eulerAnglesToPack.set(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
   }

   public void getQuaternion(Quat4d quaternionToPack)
   {
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(quaternionToPack, yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
   }

   public void getMatrix3d(Matrix3d rotationMatrixToPack)
   {
      RotationFunctions.setYawPitchRoll(rotationMatrixToPack, yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
   }

   public void getFrameOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      orientationToPack.setYawPitchRoll(yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
   }

   public FrameOrientation getFrameOrientationCopy()
   {
      FrameOrientation orientation = new FrameOrientation(getReferenceFrame(), yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
      return orientation;
   }

   public void interpolate(YoFrameOrientation orientationOne, YoFrameOrientation orientationTwo, double alpha)
   {
      orientationOne.putYoValuesIntoFrameOrientation();
      orientationTwo.putYoValuesIntoFrameOrientation();

      tempFrameOrientation.setToZero(getReferenceFrame());
      tempFrameOrientation.interpolate(orientationOne.tempFrameOrientation, orientationTwo.tempFrameOrientation, alpha);

      this.set(tempFrameOrientation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   private void putYoValuesIntoFrameOrientation()
   {
      tempFrameOrientation.setToZero(getReferenceFrame());
      tempFrameOrientation.setYawPitchRoll(yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
   }

   public boolean containsNaN()
   {
      return Double.isNaN(yaw.getDoubleValue()) || Double.isNaN(pitch.getDoubleValue()) || Double.isNaN(roll.getDoubleValue());
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      yaw.addVariableChangedListener(variableChangedListener);
      pitch.addVariableChangedListener(variableChangedListener);
      roll.addVariableChangedListener(variableChangedListener);
   }

   @Override
   public String toString()
   {
      return "(yaw = " + yaw.getDoubleValue() + ", pitch = " + pitch.getDoubleValue() + ", roll = " + roll.getDoubleValue() + ")-" + getReferenceFrame().getName();
   }
}
