package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoMultipleFramesHelper;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3Waypoint extends ReferenceFrameHolder implements SO3WaypointInterface, YoMultipleFramesHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoMultipleFramesHelper multipleFramesHelper;

   private final DoubleYoVariable time;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector angularVelocity;

   private final FrameSO3Waypoint frameSO3Waypoint = new FrameSO3Waypoint();

   public YoFrameSO3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      this.multipleFramesHelper = new YoMultipleFramesHelper(createName(namePrefix, nameSuffix, ""), registry, referenceFrames);

      time = new DoubleYoVariable(createName(namePrefix, "time", nameSuffix), registry);
      orientation = new YoFrameQuaternion(createName(namePrefix, "orientation", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return multipleFramesHelper.getCurrentReferenceFrame();
         }
      };
      angularVelocity = new YoFrameVector(createName(namePrefix, "angularVelocity", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return multipleFramesHelper.getCurrentReferenceFrame();
         }
      };
   }

   public void set(SO3WaypointInterface so3Waypoint)
   {
      frameSO3Waypoint.setToZero(getReferenceFrame());
      frameSO3Waypoint.set(so3Waypoint);
      getYoValuesFromFrameSO3Waypoint();
   }

   public void set(FrameSO3Waypoint frameSO3Waypoint)
   {
      frameSO3Waypoint.setToZero(getReferenceFrame());
      frameSO3Waypoint.set(frameSO3Waypoint);
      getYoValuesFromFrameSO3Waypoint();
   }

   public void set(YoFrameSO3Waypoint yoFrameSO3Waypoint)
   {
      frameSO3Waypoint.setToZero(getReferenceFrame());
      yoFrameSO3Waypoint.getFrameSO3Waypoint(frameSO3Waypoint);
      getYoValuesFromFrameSO3Waypoint();
   }

   public void set(double time, Quat4d orientation, Vector3d angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, FrameOrientation orientation, FrameVector angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, YoFrameQuaternion orientation, YoFrameVector angularVelocity)
   {
      this.time.set(time);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time.sub(timeOffsetToSubtract);
   }

   public boolean containsNaN()
   {
      if (time.isNaN())
         return true;
      if (orientation.containsNaN())
         return true;
      if (angularVelocity.containsNaN())
         return true;

      return false;
   }

   @Override
   public double getTime()
   {
      return time.getDoubleValue();
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientation(orientationToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTuple(angularVelocityToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   public void getOrientation(YoFrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getAngularVelocity(YoFrameVector angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Return the original orientation held by this waypoint.
    */
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Return the original angularVelocity held by this waypoint.
    */
   public YoFrameVector getAngularVelocity()
   {
      return angularVelocity;
   }

   public void getFrameSO3Waypoint(FrameSO3Waypoint frameSO3WaypointToPack)
   {
      putYoValuesIntoFrameSO3Waypoint();
      frameSO3WaypointToPack.set(frameSO3Waypoint);
   }

   public void getFrameSO3WaypointIncludingFrame(FrameSO3Waypoint frameSO3WaypointToPack)
   {
      putYoValuesIntoFrameSO3Waypoint();
      frameSO3WaypointToPack.setIncludingFrame(frameSO3Waypoint);
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredReferenceFrame)
   {
      putYoValuesIntoFrameSO3Waypoint();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
      frameSO3Waypoint.changeFrame(desiredReferenceFrame);
      getYoValuesFromFrameSO3Waypoint();
   }

   @Override
   public ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame previousReferenceFrame = multipleFramesHelper.switchCurrentReferenceFrame(referenceFrame);
      setToZero();
      return previousReferenceFrame;
   }

   @Override
   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame)
   {
      return multipleFramesHelper.isReferenceFrameRegistered(referenceFrame);
   }

   @Override
   public int getNumberOfReferenceFramesRegistered()
   {
      return multipleFramesHelper.getNumberOfReferenceFramesRegistered();
   }

   @Override
   public void getRegisteredReferenceFrames(List<ReferenceFrame> referenceFramesToPack)
   {
      multipleFramesHelper.getRegisteredReferenceFrames(referenceFramesToPack);
   }

   @Override
   public void setToNaN(ReferenceFrame desiredReferenceFrame)
   {
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
      setToNaN();
   }

   public void setToNaN()
   {
      time.set(Double.NaN);
      orientation.setToNaN();
      angularVelocity.setToNaN();
   }

   public void setToZero()
   {
      time.set(0.0);
      orientation.setToZero();
      angularVelocity.setToZero();
   }

   public FrameSO3Waypoint getFrameSO3WaypointCopy()
   {
      putYoValuesIntoFrameSO3Waypoint();
      return new FrameSO3Waypoint(frameSO3Waypoint);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return multipleFramesHelper.getCurrentReferenceFrame();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   private void putYoValuesIntoFrameSO3Waypoint()
   {
      frameSO3Waypoint.setToZero(getReferenceFrame());
      frameSO3Waypoint.set(this);
   }

   private void getYoValuesFromFrameSO3Waypoint()
   {
      getYoValuesFromFrameTuple(true);
   }

   private void getYoValuesFromFrameTuple(boolean notifyListeners)
   {
      time.set(frameSO3Waypoint.getTime(), notifyListeners);
      orientation.set(frameSO3Waypoint.getOrientation(), notifyListeners);
      angularVelocity.set(frameSO3Waypoint.getAngularVelocity(), notifyListeners);
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameSO3Waypoint();

      return frameSO3Waypoint.toString();
   }

   public boolean epsilonEquals(YoFrameSO3Waypoint other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!MathTools.epsilonEquals(time.getDoubleValue(), other.time.getDoubleValue(), epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;
      return true;
   }
}
