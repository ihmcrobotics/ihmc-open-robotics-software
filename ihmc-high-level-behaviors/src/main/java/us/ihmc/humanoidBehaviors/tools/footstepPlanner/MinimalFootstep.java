package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import java.util.ArrayList;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld)
   {
      this(side, solePoseInWorld, null, null);
   }

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld, ConvexPolygon2DReadOnly foothold)
   {
      this(side, solePoseInWorld, foothold, null);
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

   public static ArrayList<MinimalFootstep> convertPairListToMinimalFoostepList(ArrayList<Pair<RobotSide, Pose3D>> pairList)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      for (Pair<RobotSide, Pose3D> pair : pairList)
      {
         minimalFootsteps.add(new MinimalFootstep(pair.getLeft(), pair.getRight()));
      }
      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> convertFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      int size = footstepDataListMessage.getFootstepDataList().size();
      for (int i = 0; i < size; i++)
      {
         FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().get(i);
         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (Point3D contactPoint : footstep.getPredictedContactPoints2d())
         {
            foothold.addVertex(contactPoint);
         }
         foothold.update();
         Pose3D pose = new Pose3D(footstep.getLocation(), footstep.getOrientation());
         minimalFootsteps.add(new MinimalFootstep(RobotSide.fromByte(footstep.getRobotSide()), pose, foothold));
      }
      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> reduceFootstepsForUIMessager(SideDependentList<PlannedFootstepReadOnly> footsteps)
   {
      ArrayList<MinimalFootstep> footstepLocations = new ArrayList<>();
      for (RobotSide side : RobotSide.values)  // this code makes the message smaller to send over the network, TODO investigate
      {
         PlannedFootstepReadOnly footstep = footsteps.get(side);
         if (footstep != null)
         {
            FramePose3D soleFramePoseToPack = new FramePose3D();
            footstep.getFootstepPose(soleFramePoseToPack);
            soleFramePoseToPack.changeFrame(ReferenceFrame.getWorldFrame());
            footstepLocations.add(new MinimalFootstep(footstep.getRobotSide(), new Pose3D(soleFramePoseToPack), footstep.getFoothold(), ""));
         }
      }
      return footstepLocations;
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
