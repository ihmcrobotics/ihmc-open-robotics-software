package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;

/**
 * @author twan
 *         Date: 6/6/13
 */
public class AverageVelocityTrajectoryTimeProvider implements  DoubleProvider
{
   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalPositionProvider;
   private final DoubleProvider averageVelocityProvider;
   private double minimumTime;

   public AverageVelocityTrajectoryTimeProvider(PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, DoubleProvider averageVelocityProvider, double minimumTime)
   {
      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;
      this.averageVelocityProvider = averageVelocityProvider;
      this.minimumTime = minimumTime;
   }

   public double getValue()
   {
      FramePoint3D initialPosition = new FramePoint3D();
      initialPositionProvider.getPosition(initialPosition);
      initialPosition.changeFrame(ReferenceFrame.getWorldFrame());

      FramePoint3D finalPosition = new FramePoint3D();
      finalPositionProvider.getPosition(finalPosition);
      finalPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double distance = initialPosition.distance(finalPosition);

      double time = distance / averageVelocityProvider.getValue();

      time = MathTools.clamp(time, minimumTime, Double.POSITIVE_INFINITY);

      return time;
   }
}
