package us.ihmc.quadrupedBasics.gait;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class QuadrupedTimedOrientedStep extends QuadrupedTimedStep
{
   private double stepYaw;

   public QuadrupedTimedOrientedStep()
   {
   }

   public QuadrupedTimedOrientedStep(RobotQuadrant robotQuadrant, FramePoint3D goalPosition, double groundClearance, TimeIntervalBasics timeInterval, double stepYaw)
   {
      super(robotQuadrant, goalPosition, groundClearance, timeInterval);
      setStepYaw(stepYaw);
   }

   public QuadrupedTimedOrientedStep(RobotQuadrant robotQuadrant, Point3DBasics goalPosition, double groundClearance, TimeIntervalBasics timeInterval, double stepYaw)
   {
      this();
      setRobotQuadrant(robotQuadrant);
      setGoalPosition(goalPosition);
      setGroundClearance(groundClearance);
      setTimeInterval(timeInterval);
      setStepYaw(stepYaw);
   }

   public QuadrupedTimedOrientedStep(QuadrupedTimedOrientedStep other)
   {
      this(other.getRobotQuadrant(), other.getGoalPositionInternal(), other.getGroundClearance(), other.getTimeInterval(), other.getStepYaw());
   }

   public void setStepYaw(double stepYaw)
   {
      this.stepYaw = stepYaw;
   }

   public double getStepYaw()
   {
      return stepYaw;
   }

   public boolean epsilonEquals(QuadrupedTimedOrientedStep other, double epsilon)
   {
      return MathTools.epsilonEquals(stepYaw, other.stepYaw, epsilon) && super.epsilonEquals(other, epsilon) && getTimeInterval()
            .epsilonEquals(other.getTimeInterval(), epsilon);
   }

   @Override
   public String toString()
   {
      String string = super.toString();
      string += "\nstartTime: " + getTimeInterval().getStartTime();
      string += "\nendTime: " + getTimeInterval().getEndTime();
      string += "\nstepYaw:" + stepYaw;
      return string;
   }
}
