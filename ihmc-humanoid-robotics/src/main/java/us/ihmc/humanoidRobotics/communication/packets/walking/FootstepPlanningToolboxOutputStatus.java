package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepPlanningToolboxOutputStatus extends SettablePacket<FootstepPlanningToolboxOutputStatus>
{
   public FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
   public byte planningResult;
   public int planId = FootstepPlanningRequestPacket.NO_PLAN_ID;

   public PlanarRegionsListMessage planarRegionsListMessage = null;

   // body path planner fields
   public Point2D[] bodyPath;
   public Pose2D lowLevelPlannerGoal;
   public Point3D[][] navigableExtrusions;

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
      this.bodyPath = bodyPath;
   }

   public void setLowLevelPlannerGoal(Pose2D lowLevelPlannerGoal)
   {
      this.lowLevelPlannerGoal = lowLevelPlannerGoal;
   }

   public void setNavigableExtrusions(Point3D[][] navigableExtrusions)
   {
      this.navigableExtrusions = navigableExtrusions;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if (planningResult != other.planningResult)
         return false;
      if (planId != other.planId)
         return false;

      if (!nullAndEpilsonCompare(planarRegionsListMessage, other.planarRegionsListMessage, epsilon))
         return false;
      if (!nullAndEpilsonCompare(lowLevelPlannerGoal, other.lowLevelPlannerGoal, epsilon))
         return false;
      if (!bothOrNeitherAreNull(bodyPath, other.bodyPath))
         return false;
      if (!bothOrNeitherAreNull(navigableExtrusions, other.navigableExtrusions))
         return false;

      if (bodyPath != null)
      {
         for (int i = 0; i < bodyPath.length; i++)
         {
            if (!bodyPath[i].epsilonEquals(other.bodyPath[i], epsilon))
               return false;
         }
      }

      if (navigableExtrusions != null)
      {
         for (int i = 0; i < navigableExtrusions.length; i++)
         {
            for (int j = 0; j < navigableExtrusions[i].length; j++)
            {
               if (!navigableExtrusions[i][j].epsilonEquals(other.navigableExtrusions[i][j], epsilon))
                  return false;
            }
         }
      }

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
      planningResult = other.planningResult;
      footstepDataList.destination = other.footstepDataList.destination;
      footstepDataList.getQueueingProperties().set(other.footstepDataList.getQueueingProperties());
      footstepDataList.footstepDataList = new ArrayList<>();
      for (FootstepDataMessage footstepData : other.footstepDataList.footstepDataList)
         footstepDataList.footstepDataList.add(new FootstepDataMessage(footstepData));
      if (other.notes != null)
         footstepDataList.notes = new String(other.notes);
      footstepDataList.defaultSwingDuration = other.footstepDataList.defaultSwingDuration;
      footstepDataList.defaultTransferDuration = other.footstepDataList.defaultTransferDuration;
      footstepDataList.uniqueId = other.footstepDataList.uniqueId;
      planarRegionsListMessage = other.planarRegionsListMessage;
      navigableExtrusions = other.navigableExtrusions;
      bodyPath = other.bodyPath;
      lowLevelPlannerGoal = other.lowLevelPlannerGoal;
      planId = other.planId;
      setPacketInformation(other);
   }

}
