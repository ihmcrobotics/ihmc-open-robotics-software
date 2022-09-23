package us.ihmc.footstepPlanning.baselinePlanner;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeInterval;

public class SimpleTimedFootstep extends SimpleFootstep
{
   private final TimeInterval timeInterval = new TimeInterval();

   public SimpleTimedFootstep()
   {
      this(RobotSide.LEFT, new FramePose3D());
   }

   public SimpleTimedFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      setRobotSide(robotSide);
      setSoleFramePose(soleFramePose);
      this.timeInterval.setInterval(0.0, 1.0);
   }

   public SimpleTimedFootstep(RobotSide robotSide, FramePose3D soleFramePose, TimeInterval timeInterval)
   {
      setRobotSide(robotSide);
      setSoleFramePose(soleFramePose);
      this.timeInterval.set(timeInterval);
   }

   public SimpleTimedFootstep(SimpleTimedFootstep other)
   {
      set(other);
   }

   public SimpleTimedFootstep(SimpleFootstep other)
   {
      super.set(other);
      this.timeInterval.setInterval(0.0, 1.0);
   }

   public void set(SimpleTimedFootstep other)
   {
      super.set(other);
      this.timeInterval.set(other.timeInterval);
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   void getTimeInterval(TimeInterval timeInterval)
   {
      timeInterval.set(this.timeInterval);
   }

   void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }
//
//   public boolean epsilonEquals(SimpleTimedFootstep otherFootstep, double epsilon)
//   {
//      if (!super.epsilonEquals(otherFootstep, epsilon))
//         return false;
//      if (!timeInterval.epsilonEquals(otherFootstep.timeInterval, epsilon))
//         return false;
//      return true;
//   }

   @Override
   public String toString()
   {
      String message = "Robot side = " + getRobotSide();
      message += "\nSole frame pose = " + getSoleFramePose();
      message += "\nFoothold = " + getFoothold();
      message += "\nTime interval  = " + timeInterval;

      return message;
   }
}
