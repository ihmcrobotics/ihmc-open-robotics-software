package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class ThreeDoFSwingFootTrajectory
{
   final private MinimumJerkTrajectory xTrajectory;
   final private MinimumJerkTrajectory yTrajectory;
   final private ParabolicTrajectory zTrajectory;
   final private MinimumJerkTrajectory xTrajectoryAdjustment;
   final private MinimumJerkTrajectory yTrajectoryAdjustment;
   final private MinimumJerkTrajectory zTrajectoryAdjustment;
   final private FramePoint initialPosition;
   final private FramePoint finalPosition;
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeInterval;
   private TimeInterval timeIntervalOfAdjustment;
   private boolean initialized;

   public ThreeDoFSwingFootTrajectory()
   {
      xTrajectory = new MinimumJerkTrajectory();
      yTrajectory = new MinimumJerkTrajectory();
      zTrajectory = new ParabolicTrajectory();
      xTrajectoryAdjustment = new MinimumJerkTrajectory();
      yTrajectoryAdjustment = new MinimumJerkTrajectory();
      zTrajectoryAdjustment = new MinimumJerkTrajectory();
      finalPosition = new FramePoint();
      initialPosition = new FramePoint();
      position = new FramePoint();
      velocity = new FrameVector();
      acceleration = new FrameVector();
      referenceFrame = ReferenceFrame.getWorldFrame();
      timeInterval = new TimeInterval();
      timeIntervalOfAdjustment = new TimeInterval();
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
      timeIntervalOfAdjustment.setInterval(startTime, endTime);

      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());
      xTrajectory.setMoveParameters(initialPosition.getX(), 0, 0, finalPosition.getX(), 0, 0, timeInterval.getDuration());
      yTrajectory.setMoveParameters(initialPosition.getY(), 0, 0, finalPosition.getY(), 0, 0, timeInterval.getDuration());
      zTrajectory.setMoveParameters(initialPosition.getZ(), midwayPositionZ, finalPosition.getZ(), timeInterval.getDuration());
      xTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      yTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      zTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      initialized = true;

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
      computeTrajectory(0);
   }

   public void adjustTrajectory(FramePoint finalPosition, double currentTime)
   {
      computeTrajectory(currentTime);
      timeIntervalOfAdjustment.setStartTime(Math.min(Math.max(currentTime, timeInterval.getStartTime()), timeInterval.getEndTime()));
      xTrajectoryAdjustment.setMoveParameters(xTrajectoryAdjustment.getPosition(), xTrajectoryAdjustment.getVelocity(), xTrajectoryAdjustment.getAcceleration(), finalPosition.getX() - this.finalPosition.getX(), (finalPosition.getX() - this.finalPosition.getX()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      yTrajectoryAdjustment.setMoveParameters(yTrajectoryAdjustment.getPosition(), yTrajectoryAdjustment.getVelocity(), yTrajectoryAdjustment.getAcceleration(), finalPosition.getY() - this.finalPosition.getY(), (finalPosition.getY() - this.finalPosition.getY()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      zTrajectoryAdjustment.setMoveParameters(zTrajectoryAdjustment.getPosition(), zTrajectoryAdjustment.getVelocity(), zTrajectoryAdjustment.getAcceleration(), finalPosition.getZ() - this.finalPosition.getZ(), 0, 0, timeIntervalOfAdjustment.getDuration());
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("parameters must be initialized before computing trajectory");

      xTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      yTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      zTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());

      xTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      yTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      zTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());

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