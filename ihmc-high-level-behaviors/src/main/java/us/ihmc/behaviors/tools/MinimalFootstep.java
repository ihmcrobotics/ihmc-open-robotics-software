package us.ihmc.behaviors.tools;

import java.util.ArrayList;
import java.util.List;

import behavior_msgs.MinimalFootstepListMessage;
import behavior_msgs.MinimalFootstepMessage;
import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import controller_msgs.FootstepQueueStatusMessage;
import controller_msgs.FootstepStatusMessage;
import controller_msgs.QueuedFootstepStatusMessage;
import ihmc_common_msgs.Point2DMessage;
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

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld, String description)
   {
      this(side, solePoseInWorld, null, description);
   }

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld)
   {
      this(side, solePoseInWorld, null, null);
   }

   public MinimalFootstep(RobotSide side, Pose3DBasics solePoseInWorld, ConvexPolygon2DReadOnly foothold)
   {
      this(side, solePoseInWorld, foothold, null);
   }

   public MinimalFootstep(FootstepStatusMessage footstepStatusMessageToCopy)
   {
      this(RobotSide.fromByte(footstepStatusMessageToCopy.getRobotSide()),
           new Pose3D(footstepStatusMessageToCopy.getActualFootPositionInWorld().getPoint(),
                      footstepStatusMessageToCopy.getActualFootOrientationInWorld().getQuaternion()),
           null,
           null);
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

   public static ArrayList<MinimalFootstep> convertFootstepQueueMessage(FootstepQueueStatusMessage queueStatusMessage, String description)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      int size = queueStatusMessage.getQueuedFootstepList().size();

      for (int i = 0; i < size; i++)
      {
         QueuedFootstepStatusMessage queuedFootstep = queueStatusMessage.getQueuedFootstepList().get(i);
         Pose3D pose = new Pose3D(queuedFootstep.getLocation().getPoint(), queuedFootstep.getOrientation().getQuaternion());
         minimalFootsteps.add(new MinimalFootstep(RobotSide.fromByte(queuedFootstep.getRobotSide()), pose, i == size - 1 ? description : ""));
      }

      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> convertPairListToMinimalFoostepList(ArrayList<Pair<RobotSide, Pose3D>> pairList, String description)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      for (int i = 0; i < pairList.size(); i++)
      {
         Pair<RobotSide, Pose3D> pair = pairList.get(i);
         minimalFootsteps.add(new MinimalFootstep(pair.getLeft(), pair.getRight(), i == pairList.size() - 1 ? description : ""));
      }
      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> convertFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage, String description)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      int size = footstepDataListMessage.getFootstepDataList().size();
      for (int i = 0; i < size; i++)
      {
         FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().get(i);
         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (int cp = 0; cp < footstep.getPredictedContactPoints2d().size(); cp++)
         {
            us.ihmc.euclid.tuple3D.Point3D contactPoint = footstep.getPredictedContactPoints2d().get(cp).getPoint();
            foothold.addVertex(contactPoint);
         }
         foothold.update();
         Pose3D pose = new Pose3D(footstep.getLocation().getPoint(), footstep.getOrientation().getQuaternion());
         minimalFootsteps.add(new MinimalFootstep(RobotSide.fromByte(footstep.getRobotSide()), pose, foothold, i == size - 1 ? description : ""));
      }
      return minimalFootsteps;
   }

   public static MinimalFootstepListMessage convertToMinimalFootstepListMessage(FootstepDataListMessage footstepDataListMessage, String description)
   {
      MinimalFootstepListMessage minimalFootsteps = new MinimalFootstepListMessage();
      int size = footstepDataListMessage.getFootstepDataList().size();
      for (int i = 0; i < size; i++)
      {
         FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().get(i);
         MinimalFootstepMessage minimalFootstep = minimalFootsteps.getMinimalFootsteps().add();
         for (int cp = 0; cp < footstep.getPredictedContactPoints2d().size(); cp++)
         {
            us.ihmc.euclid.tuple3D.Point3D contactPoint = footstep.getPredictedContactPoints2d().get(cp).getPoint();
            Point2DMessage vertex = minimalFootstep.getSupportPolygon().add();
            vertex.setX(contactPoint.getX());
            vertex.setY(contactPoint.getY());
         }
         minimalFootstep.setRobotSide(footstep.getRobotSide());
         minimalFootstep.getPosition().set(footstep.getLocation().getPoint());
         minimalFootstep.getOrientation().set(footstep.getOrientation().getQuaternion());
         minimalFootstep.setDescription( i == size - 1 ? description : "");
      }
      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> convertMinimalFootstepListMessage(MinimalFootstepListMessage minimalFootstepListMessage)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();
      int size = minimalFootstepListMessage.getMinimalFootsteps().size();
      for (int i = 0; i < size; i++)
      {
         MinimalFootstepMessage footstep = minimalFootstepListMessage.getMinimalFootsteps().get(i);
         ConvexPolygon2D foothold = new ConvexPolygon2D();
         for (Point2DMessage contactPoint : footstep.getSupportPolygon())
         {
            foothold.addVertex(contactPoint.getX(), contactPoint.getY());
         }
         foothold.update();
         Pose3D pose = new Pose3D(footstep.getPosition().getPoint(), footstep.getOrientation().getQuaternion());
         minimalFootsteps.add(new MinimalFootstep(RobotSide.fromByte(footstep.getRobotSide()), pose, foothold, footstep.getDescriptionAsString()));
      }
      return minimalFootsteps;
   }

   public static ArrayList<MinimalFootstep> reduceFootstepsForUIMessager(SideDependentList<PlannedFootstepReadOnly> footsteps, String description)
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
            footstepLocations.add(new MinimalFootstep(footstep.getRobotSide(), new Pose3D(soleFramePoseToPack), footstep.getFoothold(), description));
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
         footstepLocations.add(new MinimalFootstep(footstep.getRobotSide(),
                                                   new Pose3D(soleFramePoseToPack),
                                                   footstep.getFoothold(),
                                                   i == footstepPlan.getNumberOfSteps() - 1 ? description : ""));
      }
      return footstepLocations;
   }

   public static MinimalFootstepListMessage reduceFootstepPlanForUIROS2(FootstepPlan footstepPlan, String description)
   {
      MinimalFootstepListMessage footstepLocations = new MinimalFootstepListMessage();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         footstep.getFootstepPose(soleFramePoseToPack);
         soleFramePoseToPack.changeFrame(ReferenceFrame.getWorldFrame());

         MinimalFootstepMessage minimalFootstepMessage = footstepLocations.getMinimalFootsteps().add();
         minimalFootstepMessage.setRobotSide(footstep.getRobotSide().toByte());
         minimalFootstepMessage.setDescription(i == footstepPlan.getNumberOfSteps() - 1 ? description : "");
         minimalFootstepMessage.getPosition().set(soleFramePoseToPack.getPosition());
         minimalFootstepMessage.getOrientation().set(soleFramePoseToPack.getOrientation());

         packFootholdToMessage(footstep.getFoothold(), minimalFootstepMessage);
      }
      return footstepLocations;
   }

   public static void packFootholdToMessage(ConvexPolygon2DReadOnly foothold, MinimalFootstepMessage minimalFootstepMessage)
   {
      minimalFootstepMessage.getSupportPolygon().clear();
      for (int i = 0; i < foothold.getNumberOfVertices(); i++)
      {
         Point2DMessage point2DMessage = minimalFootstepMessage.getSupportPolygon().add();
         point2DMessage.setX(foothold.getVertex(i).getX());
         point2DMessage.setY(foothold.getVertex(i).getY());
      }
   }
}
