package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoMultipleFramesHelper;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSE3Waypoint extends ReferenceFrameHolder implements SE3WaypointInterface, YoMultipleFramesHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoMultipleFramesHelper multipleFramesHelper;

   private final DoubleYoVariable time;
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;
   private final YoFrameVector linearVelocity;
   private final YoFrameVector angularVelocity;

   private final FrameSE3Waypoint frameSE3Waypoint = new FrameSE3Waypoint();

   public YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;
      this.multipleFramesHelper = new YoMultipleFramesHelper(createName(namePrefix, nameSuffix, ""), registry, referenceFrames);

      time = new DoubleYoVariable(createName(namePrefix, "time", nameSuffix), registry);
      position = new YoFramePoint(createName(namePrefix, "position", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return multipleFramesHelper.getCurrentReferenceFrame();
         }
      };
      orientation = new YoFrameQuaternion(createName(namePrefix, "orientation", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return multipleFramesHelper.getCurrentReferenceFrame();
         }
      };
      linearVelocity = new YoFrameVector(createName(namePrefix, "linearVelocity", ""), nameSuffix, null, registry)
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

   public void set(SE3WaypointInterface se3Waypoint)
   {
      frameSE3Waypoint.setToZero(getReferenceFrame());
      frameSE3Waypoint.set(se3Waypoint);
      getYoValuesFromFrameSE3Waypoint();
   }

   public void set(FrameSE3Waypoint frameSE3Waypoint)
   {
      frameSE3Waypoint.setToZero(getReferenceFrame());
      frameSE3Waypoint.set(frameSE3Waypoint);
      getYoValuesFromFrameSE3Waypoint();
   }

   public void set(YoFrameSE3Waypoint yoFrameSE3Waypoint)
   {
      frameSE3Waypoint.setToZero(getReferenceFrame());
      yoFrameSE3Waypoint.getFrameSE3Waypoint(frameSE3Waypoint);
      getYoValuesFromFrameSE3Waypoint();
   }

   public void set(double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, FramePoint position, FrameOrientation orientation, FrameVector linearVelocity, FrameVector angularVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
      this.angularVelocity.set(angularVelocity);
   }

   public void set(double time, YoFramePoint position, YoFrameQuaternion orientation, YoFrameVector linearVelocity, YoFrameVector angularVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.orientation.set(orientation);
      this.linearVelocity.set(linearVelocity);
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
      if (position.containsNaN())
         return true;
      if (orientation.containsNaN())
         return true;
      if (linearVelocity.containsNaN())
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
   public void getPosition(Point3d positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getOrientation(Quat4d orientationToPack)
   {
      orientation.get(orientationToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   @Override
   public void getAngularVelocity(Vector3d angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTuple(positionToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientation(orientationToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTuple(linearVelocityToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTuple(angularVelocityToPack);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTupleIncludingFrame(linearVelocityToPack);
   }

   public void getAngularVelocityIncludingFrame(FrameVector angularVelocityToPack)
   {
      angularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getOrientation(YoFrameQuaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getLinearVelocity(YoFrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   public void getAngularVelocity(YoFrameVector angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }

   /**
    * Return the original position held by this waypoint.
    */
   public YoFramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original orientation held by this waypoint.
    */
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Return the original linearVelocity held by this waypoint.
    */
   public YoFrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   /**
    * Return the original angularVelocity held by this waypoint.
    */
   public YoFrameVector getAngularVelocity()
   {
      return angularVelocity;
   }

   public void getFrameSE3Waypoint(FrameSE3Waypoint frameSE3WaypointToPack)
   {
      putYoValuesIntoFrameSE3Waypoint();
      frameSE3WaypointToPack.set(frameSE3Waypoint);
   }

   public void getFrameSE3WaypointIncludingFrame(FrameSE3Waypoint frameSE3WaypointToPack)
   {
      putYoValuesIntoFrameSE3Waypoint();
      frameSE3WaypointToPack.setIncludingFrame(frameSE3Waypoint);
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredReferenceFrame)
   {
      putYoValuesIntoFrameSE3Waypoint();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
      frameSE3Waypoint.changeFrame(desiredReferenceFrame);
      getYoValuesFromFrameSE3Waypoint();
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
      position.setToNaN();
      orientation.setToNaN();
      linearVelocity.setToNaN();
      angularVelocity.setToNaN();
   }

   public void setToZero()
   {
      time.set(0.0);
      position.setToZero();
      orientation.setToZero();
      linearVelocity.setToZero();
      angularVelocity.setToZero();
   }

   public FrameSE3Waypoint getFrameSE3WaypointCopy()
   {
      putYoValuesIntoFrameSE3Waypoint();
      return new FrameSE3Waypoint(frameSE3Waypoint);
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

   private void putYoValuesIntoFrameSE3Waypoint()
   {
      frameSE3Waypoint.setToZero(getReferenceFrame());
      frameSE3Waypoint.set(this);
   }

   private void getYoValuesFromFrameSE3Waypoint()
   {
      getYoValuesFromFrameTuple(true);
   }

   private void getYoValuesFromFrameTuple(boolean notifyListeners)
   {
      time.set(frameSE3Waypoint.getTime(), notifyListeners);
      position.set(frameSE3Waypoint.getPosition(), notifyListeners);
      orientation.set(frameSE3Waypoint.getOrientation(), notifyListeners);
      linearVelocity.set(frameSE3Waypoint.getLinearVelocity(), notifyListeners);
      angularVelocity.set(frameSE3Waypoint.getAngularVelocity(), notifyListeners);
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameSE3Waypoint();
      return frameSE3Waypoint.toString();
   }

   public boolean epsilonEquals(YoFrameSE3Waypoint other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!MathTools.epsilonEquals(time.getDoubleValue(), other.time.getDoubleValue(), epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      if (!angularVelocity.epsilonEquals(other.angularVelocity, epsilon))
         return false;
      return true;
   }
}
