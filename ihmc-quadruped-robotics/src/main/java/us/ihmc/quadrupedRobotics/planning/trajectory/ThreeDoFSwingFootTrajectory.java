package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.util.YoTimeInterval;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ThreeDoFSwingFootTrajectory
{
   private final YoPolynomial3D swingTrajectory;
   private final YoPolynomial3D swingTrajectoryAdjustment;
   private final YoDouble duration;

   final private FramePoint3D initialPosition;

   private static final FrameVector3D zeroVector = new FrameVector3D();

   final private FramePoint3D finalPosition;
   final private FramePoint3D finalPositionAdjustment = new FramePoint3D();

   final private FramePoint3D position;
   final private FrameVector3D velocity;
   final private FrameVector3D acceleration;

   private final TimeInterval timeInterval;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeIntervalAdjustment;
   private boolean initialized;

   private final YoGraphicPolynomial3D trajectoryViz;

   public ThreeDoFSwingFootTrajectory(String prefix, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      swingTrajectory = new YoPolynomial3D(prefix + "SwingTrajectory", 4, registry);
      swingTrajectoryAdjustment = new YoPolynomial3D(prefix + "SwingTrajectoryAdjustment", 6, registry);

      duration = new YoDouble(prefix + "Duration", registry);
      finalPosition = new FramePoint3D();
      initialPosition = new FramePoint3D();
      position = new FramePoint3D();
      velocity = new FrameVector3D();
      acceleration = new FrameVector3D();
      referenceFrame = ReferenceFrame.getWorldFrame();
      timeInterval = new TimeInterval();
      timeIntervalAdjustment = new TimeInterval();
      initialized = false;

      if (graphicsListRegistry != null)
      {
         trajectoryViz = new YoGraphicPolynomial3D(prefix + "Trajectory", swingTrajectory, duration, 0.01, 50, 8, registry);
         graphicsListRegistry.registerYoGraphic(prefix + "Trajectory", trajectoryViz);

         trajectoryViz.setColorType(YoGraphicPolynomial3D.TrajectoryColorType.ACCELERATION_BASED);
      }
      else
         trajectoryViz = null;
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.showGraphic();
   }

   public void getPosition(FramePoint3D position)
   {
      position.setIncludingFrame(this.position);
   }

   public void getVelocity(FrameVector3D velocity)
   {
      velocity.setIncludingFrame(this.velocity);
   }

   public void getAcceleration(FrameVector3D acceleration)
   {
      acceleration.setIncludingFrame(this.acceleration);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void initializeTrajectory(FramePoint3D initialPosition, FramePoint3D finalPosition, double groundClearance, TimeInterval timeInterval)
   {
      initializeTrajectory(initialPosition, finalPosition, groundClearance, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public void initializeTrajectory(FramePoint3D initialPosition, FramePoint3D finalPosition, double groundClearance, double startTime, double endTime)
   {
      finalPosition.checkReferenceFrameMatch(initialPosition);
      this.initialPosition.setIncludingFrame(initialPosition);
      this.finalPosition.setIncludingFrame(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();

      timeInterval.setInterval(startTime, endTime);
      duration.set(timeInterval.getDuration());
      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());

      swingTrajectory.getYoPolynomialX().setCubic(0, timeInterval.getDuration(), initialPosition.getX(), 0, finalPosition.getX(), 0);
      swingTrajectory.getYoPolynomialY().setCubic(0, timeInterval.getDuration(), initialPosition.getY(), 0, finalPosition.getY(), 0);
      swingTrajectory.getYoPolynomialZ().setQuadraticUsingIntermediatePoint(0, 0.5 * timeInterval.getDuration(), timeInterval.getDuration(), initialPosition.getZ(), midwayPositionZ,
            finalPosition.getZ());

      timeIntervalAdjustment.setInterval(startTime, endTime);
      swingTrajectoryAdjustment.getYoPolynomialX().setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);
      swingTrajectoryAdjustment.getYoPolynomialY().setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);
      swingTrajectoryAdjustment.getYoPolynomialZ().setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);

      initialized = true;
      computeTrajectory(0);
      visualize();
   }

   public void adjustTrajectory(FramePoint3D finalPosition, double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("parameters must be initialized before adjusting trajectory");
      }
      currentTime = MathTools.clamp(currentTime, timeInterval.getStartTime(), timeInterval.getEndTime());

      swingTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());

      timeIntervalAdjustment.setStartTime(currentTime);
      timeIntervalAdjustment.setEndTime(timeInterval.getEndTime());

      finalPositionAdjustment.sub(finalPosition, this.finalPosition);

      swingTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), swingTrajectoryAdjustment.getPosition(), swingTrajectoryAdjustment.getVelocity(),
                                           swingTrajectoryAdjustment.getAcceleration(), finalPositionAdjustment, zeroVector, zeroVector);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("parameters must be initialized before computing trajectory");
      }
      currentTime = MathTools.clamp(currentTime, timeInterval.getStartTime(), timeInterval.getEndTime());

      swingTrajectory.compute(currentTime - timeInterval.getStartTime());
      swingTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());

      position.set(swingTrajectory.getPosition());
      position.add(swingTrajectoryAdjustment.getPosition());

      velocity.set(swingTrajectory.getVelocity());
      velocity.add(swingTrajectoryAdjustment.getVelocity());

      acceleration.set(swingTrajectory.getAcceleration());
      acceleration.add(swingTrajectoryAdjustment.getAcceleration());
   }
}