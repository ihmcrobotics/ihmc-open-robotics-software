package us.ihmc.robotics.math.trajectories.waypoints;

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

public class YoFrameEuclideanTrajectoryPoint extends ReferenceFrameHolder
      implements EuclideanTrajectoryPointInterface<YoFrameEuclideanTrajectoryPoint>, YoMultipleFramesHolder
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoMultipleFramesHelper multipleFramesHelper;

   private final DoubleYoVariable time;
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;

   private final FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   public YoFrameEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
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

   @Override
   public void setTime(double time)
   {
      this.time.set(time);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      frameEuclideanTrajectoryPoint.setToZero(getReferenceFrame());
      frameEuclideanTrajectoryPoint.set(euclideanTrajectoryPoint);
      getYoValuesFromFrameEuclideanTrajectoryPoint();
   }

   public void set(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      frameEuclideanTrajectoryPoint.setToZero(getReferenceFrame());
      frameEuclideanTrajectoryPoint.set(frameEuclideanTrajectoryPoint);
      getYoValuesFromFrameEuclideanTrajectoryPoint();
   }

   @Override
   public void set(YoFrameEuclideanTrajectoryPoint yoFrameEuclideanTrajectoryPoint)
   {
      frameEuclideanTrajectoryPoint.setToZero(getReferenceFrame());
      yoFrameEuclideanTrajectoryPoint.getFrameEuclideanTrajectoryPoint(frameEuclideanTrajectoryPoint);
      getYoValuesFromFrameEuclideanTrajectoryPoint();
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

   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      time.add(timeOffsetToAdd);
   }

   @Override
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time.sub(timeOffsetToSubtract);
   }

   @Override
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
    * Return the original position held by this trajectory point.
    */
   public YoFramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original linearVelocity held by this trajectory point.
    */
   public YoFrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   public void getFrameEuclideanTrajectoryPoint(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPointToPack)
   {
      putYoValuesIntoFrameEuclideanTrajectoryPoint();
      frameEuclideanTrajectoryPointToPack.set(frameEuclideanTrajectoryPoint);
   }

   public void getFrameEuclideanTrajectoryPointIncludingFrame(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPointToPack)
   {
      putYoValuesIntoFrameEuclideanTrajectoryPoint();
      frameEuclideanTrajectoryPointToPack.setIncludingFrame(frameEuclideanTrajectoryPoint);
   }

   @Override
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      multipleFramesHelper.registerReferenceFrame(newReferenceFrame);
   }

   @Override
   public void changeFrame(ReferenceFrame desiredReferenceFrame)
   {
      putYoValuesIntoFrameEuclideanTrajectoryPoint();
      multipleFramesHelper.switchCurrentReferenceFrame(desiredReferenceFrame);
      frameEuclideanTrajectoryPoint.changeFrame(desiredReferenceFrame);
      getYoValuesFromFrameEuclideanTrajectoryPoint();
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

   public FrameEuclideanTrajectoryPoint getFrameEuclideanTrajectoryPointCopy()
   {
      putYoValuesIntoFrameEuclideanTrajectoryPoint();
      return new FrameEuclideanTrajectoryPoint(frameEuclideanTrajectoryPoint);
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

   private void putYoValuesIntoFrameEuclideanTrajectoryPoint()
   {
      frameEuclideanTrajectoryPoint.setToZero(getReferenceFrame());
      frameEuclideanTrajectoryPoint.set(this);
   }

   private void getYoValuesFromFrameEuclideanTrajectoryPoint()
   {
      getYoValuesFromFrameTuple(true);
   }

   private void getYoValuesFromFrameTuple(boolean notifyListeners)
   {
      time.set(frameEuclideanTrajectoryPoint.getTime(), notifyListeners);
      position.set(frameEuclideanTrajectoryPoint.getPosition(), notifyListeners);
      linearVelocity.set(frameEuclideanTrajectoryPoint.getLinearVelocity(), notifyListeners);
   }

   @Override
   public boolean epsilonEquals(YoFrameEuclideanTrajectoryPoint other, double epsilon)
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

   @Override
   public String toString()
   {
      putYoValuesIntoFrameEuclideanTrajectoryPoint();
      return frameEuclideanTrajectoryPoint.toString();
   }
}
