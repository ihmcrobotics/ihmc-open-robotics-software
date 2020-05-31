package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepForUI
{
   private final RobotSide side;
   private final Pose3D solePoseInWorld;
   private final String description;

   public FootstepForUI(RobotSide side, Pose3D solePoseInWorld, String description)
   {
      this.side = side;
      this.solePoseInWorld = solePoseInWorld;
      this.description = description;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public Pose3D getSolePoseInWorld()
   {
      return solePoseInWorld;
   }

   public String getDescription()
   {
      return description;
   }
}
