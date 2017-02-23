package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createName;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class YoFrameTrajectoryPoint<Y extends YoFrameTrajectoryPoint<Y, F, S>, F extends FrameTrajectoryPoint<F, S>, S extends Transformable & TrajectoryPointInterface<S>>
      extends YoFrameWaypoint<Y, F, S> implements TrajectoryPointInterface<Y>
{
   protected final DoubleYoVariable time;

   public YoFrameTrajectoryPoint(F frameTrajectoryPoint, String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(frameTrajectoryPoint, namePrefix, nameSuffix, registry, referenceFrames);
      time = new DoubleYoVariable(createName(namePrefix, "time", nameSuffix), registry);
   }

   @Override
   public final void setTime(double time)
   {
      this.time.set(time);
   }

   public final void setTime(TrajectoryPointInterface<?> trajectoryPoint)
   {
      time.set(trajectoryPoint.getTime());
   }

   @Override
   public final void addTimeOffset(double timeOffsetToAdd)
   {
      time.add(timeOffsetToAdd);
   }

   @Override
   public final void subtractTimeOffset(double timeOffsetToSubtract)
   {
      time.sub(timeOffsetToSubtract);
   }

   @Override
   public final void set(S simpleTrajectoryPoint)
   {
      setTime(simpleTrajectoryPoint);
      super.set(simpleTrajectoryPoint);
   }

   @Override
   public final void setIncludingFrame(ReferenceFrame referenceFrame, S simpleTrajectoryPoint)
   {
      setTime(simpleTrajectoryPoint);
      super.setIncludingFrame(referenceFrame, simpleTrajectoryPoint);
   }

   @Override
   public final void set(F frameTrajectoryPoint)
   {
      setTime(frameTrajectoryPoint);
      super.set(frameTrajectoryPoint);
   }

   @Override
   public final void setIncludingFrame(F frameTrajectoryPoint)
   {
      setTime(frameTrajectoryPoint);
      super.setIncludingFrame(frameTrajectoryPoint);
   }

   @Override
   public final void set(Y other)
   {
      setTime(other);
      super.set(other);
   }

   @Override
   public final void setIncludingFrame(Y other)
   {
      setTime(other);
      super.setIncludingFrame(other);
   }

   public final void setTimeToZero()
   {
      time.set(0.0);
   }

   public final void setTimeToNaN()
   {
      time.set(Double.NaN);
   }

   @Override
   public final double getTime()
   {
      return time.getDoubleValue();
   }

   @Override
   public final boolean epsilonEquals(Y other, double epsilon)
   {
      if (!MathTools.epsilonEquals(getTime(), other.getTime(), epsilon))
         return false;
      return super.epsilonEquals(other, epsilon);
   }
}
