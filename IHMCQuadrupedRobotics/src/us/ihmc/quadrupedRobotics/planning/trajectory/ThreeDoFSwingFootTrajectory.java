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
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeInterval;
   private boolean initialized;

   public ThreeDoFSwingFootTrajectory()
   {
      xTrajectory = new MinimumJerkTrajectory();
      yTrajectory = new MinimumJerkTrajectory();
      zTrajectory = new ParabolicTrajectory();
      position = new FramePoint();
      velocity = new FrameVector();
      acceleration = new FrameVector();
      referenceFrame = ReferenceFrame.getWorldFrame();
      timeInterval = new TimeInterval();
      initialized = false;
   }

   public double getStartTime()
   {
      return timeInterval.getStartTime();
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
      timeInterval.setInterval(startTime, endTime);
      double duration = timeInterval.getDuration();

      initialPosition.checkReferenceFrameMatch(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();

      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());
      xTrajectory.setMoveParameters(initialPosition.getX(), 0, 0, finalPosition.getX(), 0, 0, timeInterval.getDuration());
      yTrajectory.setMoveParameters(initialPosition.getY(), 0, 0, finalPosition.getY(), 0, 0, timeInterval.getDuration());
      zTrajectory.setMoveParameters(initialPosition.getZ(), midwayPositionZ, finalPosition.getZ(), timeInterval.getDuration());
      initialized = true;

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
      computeTrajectory(0);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("parameters must be initialized before computing trajectory");

      xTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      yTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      zTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());

      position.setX(xTrajectory.getPosition());
      position.setY(yTrajectory.getPosition());
      position.setZ(zTrajectory.getPosition());

      velocity.setX(xTrajectory.getVelocity());
      velocity.setY(yTrajectory.getVelocity());
      velocity.setZ(zTrajectory.getVelocity());

      acceleration.setX(xTrajectory.getAcceleration());
      acceleration.setY(yTrajectory.getAcceleration());
      acceleration.setZ(zTrajectory.getAcceleration());
   }
}