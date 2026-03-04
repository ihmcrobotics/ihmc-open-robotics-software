package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class QuickFootstep
{
   private final RobotSide swingSide;
   private final Pose3D swingEnd = new Pose3D();
   private final double swingDistance;

   public QuickFootstep(RobotSide swingSide, Pose3D swingEnd, double swingDistance)
   {
      this.swingSide = swingSide;
      this.swingEnd.set(swingEnd);
      this.swingDistance = swingDistance;
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   public Pose3D getSwingEnd()
   {
      return swingEnd;
   }

   public double getSwingDistance()
   {
      return swingDistance;
   }
}
