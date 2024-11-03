package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class C1ContinuousTrajectorySmootherTest
{
   private static final double epsilon = 1e-3;
   private static final boolean visualize = false;

   private SimulationConstructionSet scs;
   private FramePolynomial3D trajectory;
   private C1ContinuousTrajectorySmoother smoothedTrajectory;

   @BeforeEach
   public void setup()
   {
      trajectory = new FramePolynomial3D(4, ReferenceFrame.getWorldFrame());
      YoRegistry testRegistry = new YoRegistry("test");
      smoothedTrajectory = new C1ContinuousTrajectorySmoother("smoothed", trajectory, testRegistry);

      new DefaultParameterReader().readParametersInRegistry(testRegistry);

      if (visualize)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         Robot testRobot = new Robot("dummy");
         testRobot.setController(new VisualizerController(trajectory, smoothedTrajectory, graphicsListRegistry));

         scs = new SimulationConstructionSet(testRobot);
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setGroundVisible(false);

         scs.startOnAThread();

         ThreadTools.sleepSeconds(2.0);
      }
   }

   @AfterEach
   public void destroy()
   {
      trajectory = null;
      smoothedTrajectory = null;
      scs = null;
   }

   @Test
   public void testTrackingCubicTrajectoryWithNoError()
   {
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
      smoothedTrajectory.initialize();
      smoothedTrajectory.updateErrorDynamicsAtTime(0.0, startPoint, startVelocity);

      if (visualize)
      {
         scs.simulate(1);
         ThreadTools.sleepForever();
      }



      for (double time = 0.0; time <= duration; time += 0.001)
      {
         trajectory.compute(time);
         smoothedTrajectory.compute(time);

         String failure = " Failed at time " + time;

         EuclidFrameTestTools.assertGeometricallyEquals("position" + failure, trajectory.getPosition(), smoothedTrajectory.getPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("velocity" + failure, trajectory.getVelocity(), smoothedTrajectory.getVelocity(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("acceleration" + failure, trajectory.getAcceleration(), smoothedTrajectory.getAcceleration(), epsilon);
      }
   }

   @Test
   public void testTrackingCubicTrajectoryWithError() throws UnreasonableAccelerationException
   {
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
      smoothedTrajectory.initialize();
      smoothedTrajectory.updateErrorDynamicsAtTime(0.0, startPoint, startVelocity);


      if (visualize)
      {
         for (int i = 0; i < 5; i++)
         {
            scs.simulateOneRecordStep();
            ThreadTools.sleepSeconds(0.1);
         }
         ThreadTools.sleepSeconds(0.5);
      }

      double time = 0.0;
      for (; time <= 0.5 * duration; time += 0.001)
      {
         trajectory.compute(time);
         smoothedTrajectory.compute(time);

         String failure = " Failed at time " + time;

         EuclidFrameTestTools.assertGeometricallyEquals("position" + failure, trajectory.getPosition(), smoothedTrajectory.getPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("velocity" + failure, trajectory.getVelocity(), smoothedTrajectory.getVelocity(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("acceleration" + failure, trajectory.getAcceleration(), smoothedTrajectory.getAcceleration(), epsilon);
      }
      LogTools.info("updating the trajectory");


      endPoint.add(positionChange);
      FramePoint3D desiredPosition = new FramePoint3D(trajectory.getPosition());
      FrameVector3D desiredVelocity = new FrameVector3D(trajectory.getVelocity());
      trajectory.setCubic(0.0, duration, startPoint, startVelocity, endPoint, endVelocity);
      smoothedTrajectory.updateErrorDynamicsAtTime(time, desiredPosition, desiredVelocity);
      LogTools.info("updated the trajectory");

      if (visualize)
      {
         for (int i = 0; i < 5; i++)
         {
            scs.simulateOneRecordStep();
            ThreadTools.sleepSeconds(0.1);
         }
         ThreadTools.sleepForever();
      }

      FrameVector3D positionErrorWhenChanged = new FrameVector3D(smoothedTrajectory.getPositionErrorWhenStartingCancellation());
      FrameVector3D velocityErrorWhenChanged = new FrameVector3D(smoothedTrajectory.getVelocityErrorWhenStartingCancellation());


      for (; time <= duration; time += 0.001)
      {
         trajectory.compute(time);
         smoothedTrajectory.compute(time);

         String failure = " Failed at time " + time;

         FrameVector3D positionError = new FrameVector3D();
         positionError.sub(smoothedTrajectory.getPosition(), trajectory.getPosition());

         EuclidFrameTestTools.assertEquals(failure, positionError, smoothedTrajectory.getReferencePositionError(), 1e-5);
         assertTrue(failure + ", error should decrease from " + positionErrorWhenChanged.length() + ", but increased to " + positionError.length(),
                    positionError.length() < positionErrorWhenChanged.length() + 1e-3);
//         EuclidFrameTestTools.assertGeometricallyEquals("position" + failure, trajectory.getPosition(), smoothedTrajectory.getPosition(), epsilon);
//         EuclidFrameTestTools.assertGeometricallyEquals("velocity" + failure, trajectory.getVelocity(), smoothedTrajectory.getVelocity(), epsilon);
//         EuclidFrameTestTools.assertGeometricallyEquals("acceleration" + failure, trajectory.getAcceleration(), smoothedTrajectory.getAcceleration(), epsilon);
      }

      trajectory.compute(duration);
      smoothedTrajectory.compute(duration);

      EuclidFrameTestTools.assertGeometricallyEquals("Final position", trajectory.getPosition(), smoothedTrajectory.getPosition(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("Final velocity", trajectory.getVelocity(), smoothedTrajectory.getVelocity(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("Final acceleration", trajectory.getAcceleration(), smoothedTrajectory.getAcceleration(), epsilon);

   }

   private static class VisualizerController implements RobotController
   {
      private static final int numberOfBalls = 50;
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final FramePolynomial3D trajectory;
      private final C1ContinuousTrajectorySmoother smoothedTrajectory;

      private final BagOfBalls trajectoryViz;
      private final BagOfBalls smoothedTrajectoryViz;

      public VisualizerController(FramePolynomial3D trajectory, C1ContinuousTrajectorySmoother smoothedTrajectory, YoGraphicsListRegistry graphicsListRegistry)
      {
         this.trajectory = trajectory;
         this.smoothedTrajectory = smoothedTrajectory;

         trajectoryViz = new BagOfBalls(numberOfBalls, 0.01, "trajectoryViz", YoAppearance.Black(), registry, graphicsListRegistry);
         smoothedTrajectoryViz = new BagOfBalls(numberOfBalls, 0.01, "smoothedTrajectoryViz", YoAppearance.Blue(), registry, graphicsListRegistry);
      }

      @Override
      public void doControl()
      {
         LogTools.info("update");
         trajectoryViz.reset();
         smoothedTrajectoryViz.reset();
         for (double time = 0.0; time <= trajectory.getDuration(); time += trajectory.getDuration() / numberOfBalls)
         {
            trajectory.compute(time);
            trajectoryViz.setBall(trajectory.getPosition());

            smoothedTrajectory.compute(time);
            smoothedTrajectoryViz.setBall(smoothedTrajectory.getPosition());
         }
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }
}
