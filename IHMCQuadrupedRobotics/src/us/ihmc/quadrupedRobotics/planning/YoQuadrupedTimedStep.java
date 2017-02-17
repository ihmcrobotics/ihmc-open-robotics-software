package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.YoTimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class YoQuadrupedTimedStep extends QuadrupedTimedStep
{
   private final EnumYoVariable<RobotQuadrant> robotQuadrant;
   private final YoTimeInterval timeInterval;
   private final DoubleYoVariable groundClearance;
   private final YoFramePoint goalPosition;

   public YoQuadrupedTimedStep(String prefix, YoVariableRegistry registry)
   {
      super();
      this.robotQuadrant = new EnumYoVariable<>(prefix + "RobotQuadrant", registry, RobotQuadrant.class);
      this.groundClearance = new DoubleYoVariable(prefix + "GroundClearance", registry);
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
   public void getGoalPosition(FramePoint goalPosition)
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
   public void setGoalPosition(FramePoint goalPosition)
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

