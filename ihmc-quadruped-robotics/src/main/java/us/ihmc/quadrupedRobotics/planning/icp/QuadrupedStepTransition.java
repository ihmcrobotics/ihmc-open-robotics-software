package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

class QuadrupedStepTransition
{
   private QuadrupedStepTransitionType type;
   private RobotQuadrant robotQuadrant;
   private final Point3D solePosition;
   private double time;

   public QuadrupedStepTransition()
   {
      time = Double.MAX_VALUE;
      type = QuadrupedStepTransitionType.LIFT_OFF;
      solePosition = new Point3D();
      robotQuadrant = RobotQuadrant.FRONT_LEFT;
   }

   public void setTransitionTime(double time)
   {
      this.time = time;
   }

   public void setTransitionType(QuadrupedStepTransitionType type)
   {
      this.type = type;
   }

   public void setSolePosition(Point3DReadOnly solePosition)
   {
      this.solePosition.set(solePosition);
   }

   public void setRobotQuadrant(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
   }

   public double getTransitionTime()
   {
      return time;
   }

   public Point3DReadOnly getSolePosition()
   {
      return solePosition;
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public QuadrupedStepTransitionType getTransitionType()
   {
      return type;
   }
}
