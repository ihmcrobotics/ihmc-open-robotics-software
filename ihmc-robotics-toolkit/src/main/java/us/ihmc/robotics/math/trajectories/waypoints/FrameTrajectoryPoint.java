package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointInterface;

public class FrameTrajectoryPoint<T extends FrameTrajectoryPoint<T, S>, S extends TrajectoryPointInterface<S>>
      extends FrameGeometryObject<T, S> implements TrajectoryPointInterface<T>
{
   private final S geometryObject;
   
   protected FrameTrajectoryPoint(S simpleTrajectoryPoint)
   {
      super(simpleTrajectoryPoint);
      geometryObject = getGeometryObject();
   }

   @Override
   public final void setTime(double time)
   {
      geometryObject.setTime(time);
   }

   @Override
   public final void addTimeOffset(double timeOffsetToAdd)
   {
      geometryObject.addTimeOffset(timeOffsetToAdd);
   }

   @Override
   public final void subtractTimeOffset(double timeOffsetToSubtract)
   {
      geometryObject.subtractTimeOffset(timeOffsetToSubtract);
   }

   @Override
   public final void setTimeToZero()
   {
      geometryObject.setTimeToZero();
   }

   @Override
   public final void setTimeToNaN()
   {
      geometryObject.setTimeToNaN();
   }

   @Override
   public final double getTime()
   {
      return geometryObject.getTime();
   }
}
