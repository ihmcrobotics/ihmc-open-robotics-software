package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class C1ContinuousTrajectorySmootherTest
{
   private static final double epsilon = 1e-3;
   private static final boolean visualize = true;

   @Test
   public void testTrackingCubicTrajectoryWithNoError()
   {
      FramePolynomial3D trajectory = new FramePolynomial3D(4, ReferenceFrame.getWorldFrame());
      YoRegistry testRegistry = new YoRegistry("test");
      C1ContinuousTrajectorySmoother smoothedTrajectory = new C1ContinuousTrajectorySmoother("smoothed", trajectory, testRegistry);

      new DefaultParameterReader().readParametersInRegistry(testRegistry);

      FrameVector3D positionChange = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.7, -0.3, 0.4);
      FrameVector3D velocityChange = new FrameVector3D(ReferenceFrame.getWorldFrame(), -0.3, 0.2, -0.3);

      FramePoint3D startPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.3, -0.2);
      FrameVector3D startVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.3, -0.2, 0.1);
      FramePoint3D endPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, 0.3, -0.2);
      FrameVector3D endVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.1, 0.3, -0.2);
      endPoint.add(startPoint, positionChange);
      endVelocity.add(startVelocity, velocityChange);

      double duration = 3.0;
      trajectory.setCubic(0.0, duration, startPoint, startVelocity, endPoint, endVelocity);
      smoothedTrajectory.updateUsingCurrentErrorAtTime(0.0);

      if (visualize)
         visualize(trajectory, smoothedTrajectory);

      for (double time = 0.0; time <= duration; time += 0.001)
      {
         trajectory.compute(time);
         smoothedTrajectory.compute(time);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(trajectory.getPosition(), smoothedTrajectory.getPosition(), epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(trajectory.getVelocity(), smoothedTrajectory.getVelocity(), epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(trajectory.getAcceleration(), smoothedTrajectory.getAcceleration(), epsilon);
      }

   }

   private static void visualize(FramePolynomial3D trajectory, C1ContinuousTrajectorySmoother smoothedTrajectory)
   {}
}
