package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ThreeDoFSwingFootTrajectory
{
   final private YoPolynomial xTrajectory;
   final private YoPolynomial yTrajectory;
   final private YoPolynomial zTrajectory;
   final private YoPolynomial xTrajectoryAdjustment;
   final private YoPolynomial yTrajectoryAdjustment;
   final private YoPolynomial zTrajectoryAdjustment;
   final private FramePoint initialPosition;
   final private FramePoint finalPosition;
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeInterval;
   private TimeInterval timeIntervalAdjustment;
   private boolean initialized;

   public ThreeDoFSwingFootTrajectory(String prefix, YoVariableRegistry registry)
   {
      xTrajectory = new YoPolynomial(prefix + "SwingTrajectoryX", 4, registry);
      yTrajectory = new YoPolynomial(prefix + "SwingTrajectoryY", 4, registry);
      zTrajectory = new YoPolynomial(prefix + "SwingTrajectoryZ", 3, registry);
      xTrajectoryAdjustment = new YoPolynomial(prefix + "SwingTrajectoryAdjustmentX", 6, registry);
      yTrajectoryAdjustment = new YoPolynomial(prefix + "SwingTrajectoryAdjustmentY", 6, registry);
      zTrajectoryAdjustment = new YoPolynomial(prefix + "SwingTrajectoryAdjustmentZ", 6, registry);
      finalPosition = new FramePoint();
      initialPosition = new FramePoint();
      position = new FramePoint();
      velocity = new FrameVector();
      acceleration = new FrameVector();
      referenceFrame = ReferenceFrame.getWorldFrame();
      timeInterval = new TimeInterval();
      timeIntervalAdjustment = new TimeInterval();
      initialized = false;
   }

   public double getStartTime()
   {
      return timeInterval.getStartTime();
   }

   public double getEndTime()
   {
      return timeInterval.getEndTime();
   }

   public void getPosition(FramePoint position)
   {
      position.setIncludingFrame(this.position);
   }

   public void getVelocity(FrameVector velocity)
   {
      velocity.setIncludingFrame(this.velocity);
   }

   public void getAcceleration(FrameVector acceleration)
   {
      acceleration.setIncludingFrame(this.acceleration);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, TimeInterval timeInterval)
   {
      initializeTrajectory(initialPosition, finalPosition, groundClearance, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, double duration)
   {
      initializeTrajectory(initialPosition, finalPosition, groundClearance, 0, duration);
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, double startTime, double endTime)
   {
      finalPosition.checkReferenceFrameMatch(initialPosition);
      this.initialPosition.setIncludingFrame(initialPosition);
      this.finalPosition.setIncludingFrame(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();

      timeInterval.setInterval(startTime, endTime);
      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());
      xTrajectory.setCubic(0, timeInterval.getDuration(), initialPosition.getX(), 0, finalPosition.getX(), 0);
      yTrajectory.setCubic(0, timeInterval.getDuration(), initialPosition.getY(), 0, finalPosition.getY(), 0);
      zTrajectory.setQuadraticUsingIntermediatePoint(0, 0.5 * timeInterval.getDuration(), timeInterval.getDuration(), initialPosition.getZ(), midwayPositionZ,
            finalPosition.getZ());

      timeIntervalAdjustment.setInterval(startTime, endTime);
      xTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);
      yTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);
      zTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), 0, 0, 0, 0, 0, 0);

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);

      initialized = true;
      computeTrajectory(0);
   }

   public void adjustTrajectory(FramePoint finalPosition, double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("parameters must be initialized before adjusting trajectory");
      }
      currentTime = MathTools.clamp(currentTime, timeInterval.getStartTime(), timeInterval.getEndTime());

      xTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());
      yTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());
      zTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());

      timeIntervalAdjustment.setStartTime(currentTime);
      timeIntervalAdjustment.setEndTime(timeInterval.getEndTime());
      double finalPositionOffsetX = finalPosition.getX() - this.finalPosition.getX();
      double finalPositionOffsetY = finalPosition.getY() - this.finalPosition.getY();
      double finalPositionOffsetZ = finalPosition.getZ() - this.finalPosition.getZ();
      xTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), xTrajectoryAdjustment.getPosition(), xTrajectoryAdjustment.getVelocity(),
            xTrajectoryAdjustment.getAcceleration(), finalPositionOffsetX, 0, 0);
      yTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), yTrajectoryAdjustment.getPosition(), yTrajectoryAdjustment.getVelocity(),
            yTrajectoryAdjustment.getAcceleration(), finalPositionOffsetY, 0, 0);
      zTrajectoryAdjustment.setQuintic(0, timeIntervalAdjustment.getDuration(), zTrajectoryAdjustment.getPosition(), zTrajectoryAdjustment.getVelocity(),
            zTrajectoryAdjustment.getAcceleration(), finalPositionOffsetZ, 0, 0);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
      {
         throw new RuntimeException("parameters must be initialized before computing trajectory");
      }
      currentTime = MathTools.clamp(currentTime, timeInterval.getStartTime(), timeInterval.getEndTime());

      xTrajectory.compute(currentTime - timeInterval.getStartTime());
      yTrajectory.compute(currentTime - timeInterval.getStartTime());
      zTrajectory.compute(currentTime - timeInterval.getStartTime());
      xTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());
      yTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());
      zTrajectoryAdjustment.compute(currentTime - timeIntervalAdjustment.getStartTime());

      position.setX(xTrajectory.getPosition() + xTrajectoryAdjustment.getPosition());
      position.setY(yTrajectory.getPosition() + yTrajectoryAdjustment.getPosition());
      position.setZ(zTrajectory.getPosition() + zTrajectoryAdjustment.getPosition());
      velocity.setX(xTrajectory.getVelocity() + xTrajectoryAdjustment.getVelocity());
      velocity.setY(yTrajectory.getVelocity() + yTrajectoryAdjustment.getVelocity());
      velocity.setZ(zTrajectory.getVelocity() + zTrajectoryAdjustment.getVelocity());
      acceleration.setX(xTrajectory.getAcceleration() + xTrajectoryAdjustment.getAcceleration());
      acceleration.setY(yTrajectory.getAcceleration() + yTrajectoryAdjustment.getAcceleration());
      acceleration.setZ(zTrajectory.getAcceleration() + zTrajectoryAdjustment.getAcceleration());
   }
}