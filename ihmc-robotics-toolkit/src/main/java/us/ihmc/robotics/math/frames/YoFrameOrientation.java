package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameOrientation implements ReferenceFrameHolder, Clearable
{
   private final YoDouble yaw, pitch, roll; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   private final double[] tempYawPitchRoll = new double[3];
   private final FrameQuaternion tempFrameOrientation = new FrameQuaternion();

   public YoFrameOrientation(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameOrientation(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      yaw = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "yaw", nameSuffix), registry);
      pitch = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "pitch", nameSuffix), registry);
      roll = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "roll", nameSuffix), registry);

      this.referenceFrame = referenceFrame;

      // frameVector = new FrameVector(frame);
   }

   public YoFrameOrientation(YoDouble yaw, YoDouble pitch, YoDouble roll, ReferenceFrame referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.referenceFrame = referenceFrame;
   }

   public void setEulerAngles(Vector3D eulerAngles)
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
   
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw);
   }
   
   public void setPitch(double pitch)
   {
      this.pitch.set(pitch);
   }
   
   public void setRoll(double roll)
   {
      this.roll.set(roll);
   }

   public void set(RotationMatrixReadOnly rotation)
   {
      tempFrameOrientation.setIncludingFrame(getReferenceFrame(), rotation);
      set(tempFrameOrientation);
   }

   public void set(QuaternionReadOnly quaternion)
   {
      set(quaternion, true);
   }

   public void set(QuaternionReadOnly quaternion, boolean notifyListeners)
   {
      tempFrameOrientation.setIncludingFrame(getReferenceFrame(), quaternion);
      set(tempFrameOrientation, notifyListeners);
   }

   public void set(RigidBodyTransform transform3D)
   {
      tempFrameOrientation.setIncludingFrame(getReferenceFrame(), transform3D.getRotationMatrix());
      set(tempFrameOrientation);
   }

   public void set(FrameQuaternionReadOnly orientation)
   {
      set(orientation, true);
   }

   public void set(FrameQuaternionReadOnly orientation, boolean notifyListeners)
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

   public void setAndMatchFrame(FrameQuaternionReadOnly orientation)
   {
      setAndMatchFrame(orientation, true);
   }

   public void setAndMatchFrame(FrameQuaternionReadOnly orientation, boolean notifyListeners)
   {
      tempFrameOrientation.setIncludingFrame(orientation);
      tempFrameOrientation.changeFrame(getReferenceFrame());
      tempFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      yaw.set(tempYawPitchRoll[0], notifyListeners);
      pitch.set(tempYawPitchRoll[1], notifyListeners);
      roll.set(tempYawPitchRoll[2], notifyListeners);
   }

   public void setAndMatchFrame(YoFrameOrientation yoFrameOrientation)
   {
      setAndMatchFrame(yoFrameOrientation, true);
   }

   public void setAndMatchFrame(YoFrameOrientation yoFrameOrientation, boolean notifyListeners)
   {
      yoFrameOrientation.getFrameOrientationIncludingFrame(tempFrameOrientation);
      tempFrameOrientation.changeFrame(getReferenceFrame());
      tempFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      yaw.set(tempYawPitchRoll[0], notifyListeners);
      pitch.set(tempYawPitchRoll[1], notifyListeners);
      roll.set(tempYawPitchRoll[2], notifyListeners);
   }
   
   /**
    * Sets the orientation of this to the origin of the passed in ReferenceFrame.
    * 
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame, boolean notifyListeners)
   {
      tempFrameOrientation.setToZero(referenceFrame);
      tempFrameOrientation.changeFrame(getReferenceFrame());
      tempFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      yaw.set(tempYawPitchRoll[0], notifyListeners);
      pitch.set(tempYawPitchRoll[1], notifyListeners);
      roll.set(tempYawPitchRoll[2], notifyListeners);
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

   @Override
   public void setToNaN()
   {
      yaw.set(Double.NaN);
      pitch.set(Double.NaN);
      roll.set(Double.NaN);
   }

   @Override
   public void setToZero()
   {
      yaw.set(0.0);
      pitch.set(0.0);
      roll.set(0.0);
   }

   public void add(YoFrameOrientation orientation)
   {
      yaw.add(orientation.getYaw());
      pitch.add(orientation.getPitch());
      roll.add(orientation.getRoll());
   }

   public void add(double yaw, double pitch, double roll)
   {
      this.yaw.add(yaw);
      this.pitch.add(pitch);
      this.roll.add(roll);
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

   public YoDouble getYaw()
   {
      return yaw;
   }

   public YoDouble getPitch()
   {
      return pitch;
   }

   public YoDouble getRoll()
   {
      return roll;
   }

   public void getEulerAngles(Vector3D eulerAnglesToPack)
   {
      eulerAnglesToPack.set(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
   }

   public void getQuaternion(Quaternion quaternionToPack)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue(), quaternionToPack);
   }

   public void getMatrix3d(RotationMatrix rotationMatrixToPack)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue(), rotationMatrixToPack);
   }

   public void getFrameOrientationIncludingFrame(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(getReferenceFrame());
      orientationToPack.setYawPitchRoll(yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
   }

   public FrameQuaternion getFrameOrientationCopy()
   {
      FrameQuaternion orientation = new FrameQuaternion(getReferenceFrame(), yaw.getDoubleValue(), pitch.getDoubleValue(), roll.getDoubleValue());
      return orientation;
   }
   
   public FrameQuaternion getFrameOrientation()
   {
      putYoValuesIntoFrameOrientation();
      return tempFrameOrientation;
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

   @Override
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
