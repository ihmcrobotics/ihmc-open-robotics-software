package us.ihmc.quadrupedRobotics.util;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.YoTimeInterval;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class YoQuadrupedTimedStep extends QuadrupedTimedStep
{
   private final YoEnum<RobotQuadrant> robotQuadrant;
   private final TimeIntervalBasics timeInterval;
   private final YoDouble groundClearance;
   private final YoFramePoint3D goalPosition;
   private final YoEnum<TrajectoryType> trajectoryType;

   public YoQuadrupedTimedStep(String prefix, YoVariableRegistry registry)
   {
      super();
      this.robotQuadrant = new YoEnum<>(prefix + "RobotQuadrant", registry, RobotQuadrant.class);
      this.groundClearance = new YoDouble(prefix + "GroundClearance", registry);
      this.goalPosition = new YoFramePoint3D(prefix + "GoalPosition", ReferenceFrame.getWorldFrame(), registry);
      this.timeInterval = new YoTimeInterval(prefix + "TimeInterval", registry);
      this.trajectoryType = YoEnum.create(prefix + "TrajectoryType", prefix + "TrajectoryType", TrajectoryType.class, registry, true);
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
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   protected Point3DBasics getGoalPositionInternal()
   {
      return goalPosition;
   }

   @Override
   public Point3DReadOnly getGoalPosition()
   {
      return goalPosition;
   }

   @Override
   public double getGroundClearance()
   {
      return this.groundClearance.getDoubleValue();
   }
   
   @Override
   public TrajectoryType getTrajectoryType()
   {
      return this.trajectoryType.getEnumValue();
   }
   
   @Override
   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType.set(trajectoryType);
   }

   @Override
   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance.set(groundClearance);
   }
}

