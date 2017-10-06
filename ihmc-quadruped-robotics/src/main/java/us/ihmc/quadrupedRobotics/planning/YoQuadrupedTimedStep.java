package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.YoTimeInterval;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class YoQuadrupedTimedStep extends QuadrupedTimedStep
{
   private final YoEnum<RobotQuadrant> robotQuadrant;
   private final YoTimeInterval timeInterval;
   private final YoDouble groundClearance;
   private final YoFramePoint goalPosition;

   public YoQuadrupedTimedStep(String prefix, YoVariableRegistry registry)
   {
      super();
      this.robotQuadrant = new YoEnum<>(prefix + "RobotQuadrant", registry, RobotQuadrant.class);
      this.groundClearance = new YoDouble(prefix + "GroundClearance", registry);
      this.goalPosition = new YoFramePoint(prefix + "GoalPosition", ReferenceFrame.getWorldFrame(), registry);
      this.timeInterval = new YoTimeInterval(prefix + "TimeInterval", registry);
   }

   @Override
   public RobotQuadrant getRobotQuadrant()
   {
      return this.robotQuadrant.getEnumValue();
   }

   @Override
   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant.set(robotQuadrant);
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return this.timeInterval;
   }

   @Override
   public void getTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.get(timeInterval);
   }

   @Override
   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   /**
    * Unsafe for external use.
    */
   @Override
   protected Point3D getGoalPosition()
   {
      return this.goalPosition.getFrameTuple().getPoint();
   }

   @Override
   public void getGoalPosition(Point3D goalPosition)
   {
      goalPosition.set(this.goalPosition.getFrameTuple().getPoint());
   }

   @Override
   public void getGoalPosition(FramePoint3D goalPosition)
   {
      ReferenceFrame originalFrame = goalPosition.getReferenceFrame();
      goalPosition.setIncludingFrame(this.goalPosition.getFrameTuple());
      goalPosition.changeFrame(originalFrame);
   }

   @Override
   public void setGoalPosition(Point3D goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   @Override
   public void setGoalPosition(FramePoint3D goalPosition)
   {
      this.goalPosition.setAndMatchFrame(goalPosition);
   }

   @Override
   public double getGroundClearance()
   {
      return this.groundClearance.getDoubleValue();
   }

   @Override
   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance.set(groundClearance);
   }
}

