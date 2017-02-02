package us.ihmc.exampleSimulations.planarWalker;

import javax.vecmath.Point3d;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.YoTimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class YoPlanarWalkerTimedStep extends PlanarWalkerTimedStep
{
   private final EnumYoVariable<RobotSide> robotSide;
   private final YoTimeInterval timeInterval;
   private final DoubleYoVariable groundClearance;
   private final YoFramePoint goalPosition;

   public YoPlanarWalkerTimedStep(String prefix, YoVariableRegistry registry)
   {
      super();
      this.robotSide = new EnumYoVariable<>(prefix + "RobotSide", registry, RobotSide.class);
      this.groundClearance = new DoubleYoVariable(prefix + "GroundClearance", registry);
      this.goalPosition = new YoFramePoint(prefix + "GoalPosition", ReferenceFrame.getWorldFrame(), registry);
      this.timeInterval = new YoTimeInterval(prefix + "TimeInterval", registry);
   }

   @Override
   public RobotSide getRobotSide()
   {
      return this.robotSide.getEnumValue();
   }

   @Override
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
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
   protected Point3d getGoalPosition()
   {
      return this.goalPosition.getFrameTuple().getPoint();
   }

   @Override
   public void getGoalPosition(Point3d goalPosition)
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
   public void setGoalPosition(Point3d goalPosition)
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
