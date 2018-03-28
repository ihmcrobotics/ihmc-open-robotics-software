package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.robotics.geometry.PlanarRegionsList;

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

   public PlanarRegionsListMessage planarRegionsListMessage = null;

   // body path planner fields
   public RecyclingArrayListPubSub<Point2D> bodyPath = new RecyclingArrayListPubSub<>(Point2D.class, Point2D::new, 5);
   public Pose2D lowLevelPlannerGoal;

   public FootstepPlanningToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public void setPlanId(int planId)
   {
      this.planId = planId;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
   }

   public void setBodyPath(Point2D[] bodyPath)
   {
      MessageTools.copyData(bodyPath, this.bodyPath);
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

      if (!nullAndEpilsonCompare(planarRegionsListMessage, other.planarRegionsListMessage, epsilon))
         return false;
      if (!nullAndEpilsonCompare(lowLevelPlannerGoal, other.lowLevelPlannerGoal, epsilon))
         return false;
      if (!bothOrNeitherAreNull(bodyPath, other.bodyPath))
         return false;

      if (!MessageTools.epsilonEquals(bodyPath, other.bodyPath, epsilon))
         return false;

      if (!footstepDataList.epsilonEquals(other.footstepDataList, epsilon))
         return false;

      return true;
   }

   private static <T extends EpsilonComparable<T>> boolean nullAndEpilsonCompare(T object1, T object2, double epsilon)
   {
      if (!bothOrNeitherAreNull(object1, object2))
         return false;
      else if (object1 != null && !object1.epsilonEquals(object2, epsilon))
         return false;
      return true;
   }

   private static boolean bothOrNeitherAreNull(Object object1, Object object2)
   {
      if (object1 == null && object2 != null)
         return false;
      else if (object1 != null && object2 == null)
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
      planarRegionsListMessage = other.planarRegionsListMessage;
      bodyPath = other.bodyPath;
      lowLevelPlannerGoal = other.lowLevelPlannerGoal;
      planId = other.planId;
      setPacketInformation(other);
   }

}
