package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class ThreeDoFMinimumJerkTrajectory
{
   final private MinimumJerkTrajectory xTrajectory;
   final private MinimumJerkTrajectory yTrajectory;
   final private MinimumJerkTrajectory zTrajectory;
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeInterval;
   private boolean initialized;

   public ThreeDoFMinimumJerkTrajectory()
   {
      xTrajectory = new MinimumJerkTrajectory();
      yTrajectory = new MinimumJerkTrajectory();
      zTrajectory = new MinimumJerkTrajectory();
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

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, TimeInterval timeInterval)
   {
      initializeTrajectory(initialPosition, finalPosition, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double duration)
   {
      initializeTrajectory(initialPosition, finalPosition, 0, duration);
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double startTime, double endTime)
   {
      timeInterval.setInterval(startTime, endTime);
      double duration = timeInterval.getDuration();

      initialPosition.checkReferenceFrameMatch(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();

      xTrajectory.setMoveParameters(initialPosition.getX(), 0, 0, finalPosition.getX(), 0, 0, duration);
      yTrajectory.setMoveParameters(initialPosition.getY(), 0, 0, finalPosition.getY(), 0, 0, duration);
      zTrajectory.setMoveParameters(initialPosition.getZ(), 0, 0, finalPosition.getZ(), 0, 0, duration);
      initialized = true;

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
      computeTrajectory(startTime);
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