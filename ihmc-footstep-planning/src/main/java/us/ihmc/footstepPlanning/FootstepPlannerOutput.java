package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public class FootstepPlannerOutput
{
   private int planId;
   private final FootstepPlan footstepPlan = new FootstepPlan();
   private BodyPathPlanningResult bodyPathPlanningResult;
   private FootstepPlanningResult footstepPlanningResult;
   private PlanarRegionsList planarRegionsList;
   private final ArrayList<Pose3D> bodyPath = new ArrayList<>();
   private final Pose3D lowLevelGoal = new Pose3D();
   private Exception exception;
   private final FootstepPlannerTimings plannerTimings = new FootstepPlannerTimings();

   public FootstepPlannerOutput()
   {
      clear();
   }

   public void clear()
   {
      planId = -1;
      footstepPlan.clear();
      bodyPathPlanningResult = null;
      footstepPlanningResult = null;
      planarRegionsList = null;
      bodyPath.clear();
      lowLevelGoal.setToNaN();
      exception = null;
      plannerTimings.clear();
   }

   public int getPlanId()
   {
      return planId;
   }

   public FootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public BodyPathPlanningResult getBodyPathPlanningResult()
   {
      return bodyPathPlanningResult;
   }

   public FootstepPlanningResult getFootstepPlanningResult()
   {
      return footstepPlanningResult;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public ArrayList<Pose3D> getBodyPath()
   {
      return bodyPath;
   }

   public Pose3D getLowLevelGoal()
   {
      return lowLevelGoal;
   }

   public Exception getException()
   {
      return exception;
   }

   public FootstepPlannerTimings getPlannerTimings()
   {
      return plannerTimings;
   }

   public void setPlanId(int planId)
   {
      this.planId = planId;
   }

   public void setBodyPathPlanningResult(BodyPathPlanningResult bodyPathPlanningResult)
   {
      this.bodyPathPlanningResult = bodyPathPlanningResult;
   }

   public void setFootstepPlanningResult(FootstepPlanningResult footstepPlanningResult)
   {
      this.footstepPlanningResult = footstepPlanningResult;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setLowLevelGoal(Pose3D lowLevelGoal)
   {
      this.lowLevelGoal.set(lowLevelGoal);
   }

   public void setException(Exception exception)
   {
      this.exception = exception;
   }

   public void setPacket(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      outputStatus.setPlanId(getPlanId());
      outputStatus.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(getFootstepPlan(), -1.0, -1.0, ExecutionMode.OVERRIDE));
      outputStatus.getBodyPath().clear();
      outputStatus.getLowLevelPlannerGoal().set(getLowLevelGoal());
      getPlannerTimings().setPacket(outputStatus.getPlannerTimings());

      if (getBodyPathPlanningResult() != null)
      {
         outputStatus.setBodyPathPlanningResult(getBodyPathPlanningResult().toByte());
      }

      if (getFootstepPlanningResult() != null)
      {
         outputStatus.setFootstepPlanningResult(getFootstepPlanningResult().toByte());
      }

      if(getException() != null)
      {
         outputStatus.setExceptionMessage(getException().toString());
         StackTraceElement[] stackTrace = getException().getStackTrace();
         if(stackTrace != null)
         {
            int numberOfElements = Math.min(exception.getStackTrace().length, 20);
            for (int i = 0; i < numberOfElements; i++)
            {
               outputStatus.getStacktrace().add(exception.getStackTrace()[i].toString());
            }
         }
      }

      if(getPlanarRegionsList() != null)
      {
         outputStatus.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getPlanarRegionsList()));
      }

      for (int i = 0; i < bodyPath.size(); i++)
      {
         outputStatus.getBodyPath().add().set(bodyPath.get(i));
      }
   }
}
