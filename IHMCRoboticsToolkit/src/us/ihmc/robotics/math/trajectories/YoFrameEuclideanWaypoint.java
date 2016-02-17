package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoMultipleFramesHelper;
import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameEuclideanWaypoint extends ReferenceFrameHolder implements EuclideanWaypointInterface, YoMultipleFramesHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoMultipleFramesHelper multipleFramesHelper;

   private final DoubleYoVariable time;
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;

   private final FrameEuclideanWaypoint frameEuclideanWaypoint = new FrameEuclideanWaypoint();

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
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
      linearVelocity = new YoFrameVector(createName(namePrefix, "linearVelocity", ""), nameSuffix, null, registry)
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return multipleFramesHelper.getCurrentReferenceFrame();
         }
      };
   }

   public void set(EuclideanWaypointInterface euclideanWaypoint)
   {
      frameEuclideanWaypoint.setToZero(getReferenceFrame());
      frameEuclideanWaypoint.set(euclideanWaypoint);
      getYoValuesFromFrameEuclideanWaypoint();
   }

   public void set(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      frameEuclideanWaypoint.setToZero(getReferenceFrame());
      frameEuclideanWaypoint.set(frameEuclideanWaypoint);
      getYoValuesFromFrameEuclideanWaypoint();
   }

   public void set(YoFrameEuclideanWaypoint yoFrameEuclideanWaypoint)
   {
      frameEuclideanWaypoint.setToZero(getReferenceFrame());
      yoFrameEuclideanWaypoint.getFrameEuclideanWaypoint(frameEuclideanWaypoint);
      getYoValuesFromFrameEuclideanWaypoint();
   }

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(double time, YoFramePoint position, YoFrameVector linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
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
      if (linearVelocity.containsNaN())
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
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTuple(positionToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTuple(linearVelocityToPack);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTupleIncludingFrame(linearVelocityToPack);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getLinearVelocity(YoFrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   /**
    * Return the original position held by this waypoint.
    */
   public YoFramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original linearVelocity held by this waypoint.
    */
   public YoFrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   public void getFrameEuclideanWaypoint(FrameEuclideanWaypoint frameEuclideanWaypointToPack)
   {
      putYoValuesIntoFrameEuclideanWaypoint();
      frameEuclideanWaypointToPack.set(frameEuclideanWaypoint);
   }

   public void getFrameEuclideanWaypointIncludingFrame(FrameEuclideanWaypoint frameEuclideanWaypointToPack)
   {
      putYoValuesIntoFrameEuclideanWaypoint();
      frameEuclideanWaypointToPack.setIncludingFrame(frameEuclideanWaypoint);
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredReferenceFrame)
   {
      putYoValuesIntoFrameEuclideanWaypoint();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
      frameEuclideanWaypoint.changeFrame(desiredReferenceFrame);
      getYoValuesFromFrameEuclideanWaypoint();
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
      linearVelocity.setToNaN();
   }

   public void setToZero()
   {
      time.set(0.0);
      position.setToZero();
      linearVelocity.setToZero();
   }

   public FrameEuclideanWaypoint getFrameEuclideanWaypointCopy()
   {
      putYoValuesIntoFrameEuclideanWaypoint();
      return new FrameEuclideanWaypoint(frameEuclideanWaypoint);
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

   private void putYoValuesIntoFrameEuclideanWaypoint()
   {
      frameEuclideanWaypoint.setToZero(getReferenceFrame());
      frameEuclideanWaypoint.set(this);
   }

   private void getYoValuesFromFrameEuclideanWaypoint()
   {
      getYoValuesFromFrameTuple(true);
   }

   private void getYoValuesFromFrameTuple(boolean notifyListeners)
   {
      time.set(frameEuclideanWaypoint.getTime(), notifyListeners);
      position.set(frameEuclideanWaypoint.getPosition(), notifyListeners);
      linearVelocity.set(frameEuclideanWaypoint.getLinearVelocity(), notifyListeners);
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameEuclideanWaypoint();
      return frameEuclideanWaypoint.toString();
   }

   public boolean epsilonEquals(YoFrameEuclideanWaypoint other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!MathTools.epsilonEquals(time.getDoubleValue(), other.time.getDoubleValue(), epsilon))
         return false;
      if (!position.epsilonEquals(other.position, epsilon))
         return false;
      if (!linearVelocity.epsilonEquals(other.linearVelocity, epsilon))
         return false;
      return true;
   }
}
