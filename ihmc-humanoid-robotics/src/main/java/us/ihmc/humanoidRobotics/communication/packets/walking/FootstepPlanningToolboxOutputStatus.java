package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class FootstepPlanningToolboxOutputStatus extends Packet<FootstepPlanningToolboxOutputStatus>
{
   public static final byte FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION = 0;
   public static final byte FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION = 1;
   public static final byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = 2;
   public static final byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = 3;
   public static final byte FOOTSTEP_PLANNING_RESULT_SNAPPING_FAILED = 4;
   public static final byte FOOTSTEP_PLANNING_RESULT_PLANNER_FAILED = 5;

   public FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
   public byte footstepPlanningResult;
   public int planId = FootstepPlanningRequestPacket.NO_PLAN_ID;

   public PlanarRegionsListMessage planarRegionsList = new PlanarRegionsListMessage();

   // body path planner fields
   public RecyclingArrayListPubSub<Point2D> bodyPath = new RecyclingArrayListPubSub<>(Point2D.class, Point2D::new, 50);
   public Pose2D lowLevelPlannerGoal = new Pose2D();

   public FootstepPlanningToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public void setPlanId(int planId)
   {
      this.planId = planId;
   }

   public void setLowLevelPlannerGoal(Pose2D lowLevelPlannerGoal)
   {
      this.lowLevelPlannerGoal = lowLevelPlannerGoal;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if (footstepPlanningResult != other.footstepPlanningResult)
         return false;
      if (planId != other.planId)
         return false;

      if (!planarRegionsList.epsilonEquals(other.planarRegionsList, epsilon))
         return false;
      if (!lowLevelPlannerGoal.epsilonEquals(other.lowLevelPlannerGoal, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(bodyPath, other.bodyPath, epsilon))
         return false;

      if (!footstepDataList.epsilonEquals(other.footstepDataList, epsilon))
         return false;

      return true;
   }

   @Override
   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      footstepPlanningResult = other.footstepPlanningResult;
      footstepDataList.destination = other.footstepDataList.destination;
      footstepDataList.getQueueingProperties().set(other.footstepDataList.getQueueingProperties());
      footstepDataList.set(other.footstepDataList);
      footstepDataList.defaultSwingDuration = other.footstepDataList.defaultSwingDuration;
      footstepDataList.defaultTransferDuration = other.footstepDataList.defaultTransferDuration;
      footstepDataList.uniqueId = other.footstepDataList.uniqueId;
      planarRegionsList = other.planarRegionsList;
      bodyPath = other.bodyPath;
      lowLevelPlannerGoal = other.lowLevelPlannerGoal;
      planId = other.planId;
      setPacketInformation(other);
   }

}
