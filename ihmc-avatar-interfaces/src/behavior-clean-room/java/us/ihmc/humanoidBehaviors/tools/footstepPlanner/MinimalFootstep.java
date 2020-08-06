package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class MinimalFootstep
{
   private final RobotSide side;
   private final Pose3DBasics solePoseInWorld;
   private final ConvexPolygon2DReadOnly foothold;
   private final String description;

   /** for Kryo */
   public MinimalFootstep()
   {
      side = null;
      solePoseInWorld = null;
      foothold = null;
      description = null;
   }

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld, ConvexPolygon2DReadOnly foothold, String description)
   {
      this.side = side;
      this.solePoseInWorld = solePoseInWorld;
      this.foothold = foothold;
      this.description = description;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public Pose3DReadOnly getSolePoseInWorld()
   {
      return solePoseInWorld;
   }

   public String getDescription()
   {
      return description;
   }

   public ConvexPolygon2DReadOnly getFoothold()
   {
      return foothold;
   }

   public static ArrayList<MinimalFootstep> reduceFootstepPlanForUIMessager(FootstepPlan footstepPlan, String description)
   {
      ArrayList<MinimalFootstep> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         footstep.getFootstepPose(soleFramePoseToPack);
         soleFramePoseToPack.changeFrame(ReferenceFrame.getWorldFrame());
         footstepLocations.add(new MinimalFootstep(footstep.getRobotSide(), new Pose3D(soleFramePoseToPack), footstep.getFoothold(), description));
      }
      return footstepLocations;
   }
}
