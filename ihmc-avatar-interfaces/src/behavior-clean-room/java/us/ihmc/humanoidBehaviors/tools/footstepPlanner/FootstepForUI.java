package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
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

   public static ArrayList<FootstepForUI> reduceFootstepPlanForUIMessager(FootstepPlan footstepPlan, String description)
   {
      ArrayList<FootstepForUI> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getPose(soleFramePoseToPack);
         soleFramePoseToPack.changeFrame(ReferenceFrame.getWorldFrame());
         footstepLocations.add(new FootstepForUI(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack), description));
      }
      return footstepLocations;
   }
}
