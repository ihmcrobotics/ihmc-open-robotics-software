package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public class FootstepPlannerOutput
{
   /**
    * ID of the request this output corresponds to
    */
   private int requestId;

   /**
    * Footstep plan, may be empty depending on the state of the planner. See {@link #footstepPlanningResult}
    */
   private final FootstepPlan footstepPlan = new FootstepPlan();

   /**
    * Body path plan result. Null if no result is available
    */
   private BodyPathPlanningResult bodyPathPlanningResult;

   /**
    * Footstep planner result. Null if no result is available
    */
   private FootstepPlanningResult footstepPlanningResult;

   /**
    * @deprecated Regions that correspond to the request message. Originally used for debugging networking
    */
   private PlanarRegionsList planarRegionsList;

   /**
    * Planned body path. Empty if planner failed
    */
   private final ArrayList<Pose3D> bodyPath = new ArrayList<>();

   /**
    * Goal pose used by the planner. This will be different from the requested goal pose if it's beyond the horizon length.
    */
   private final Pose3D goalPose = new Pose3D();

   /**
    * Any exception thrown by {@link FootstepPlanningModule#handleRequest(FootstepPlannerRequest)} is caught and saved here
    */
   private Exception exception;

   /**
    * Object to record various planner timings, helpful for debugging
    */
   private final FootstepPlannerTimings plannerTimings = new FootstepPlannerTimings();

   public FootstepPlannerOutput()
   {
      clear();
   }

   public void clear()
   {
      requestId = -1;
      footstepPlan.clear();
      bodyPathPlanningResult = null;
      footstepPlanningResult = null;
      planarRegionsList = null;
      bodyPath.clear();
      goalPose.setToNaN();
      exception = null;
      plannerTimings.clear();
   }

   public int getRequestId()
   {
      return requestId;
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

   /**
    * @deprecated
    * Use the regions from the request
    */
   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public ArrayList<Pose3D> getBodyPath()
   {
      return bodyPath;
   }

   public Pose3D getGoalPose()
   {
      return goalPose;
   }

   public Exception getException()
   {
      return exception;
   }

   public FootstepPlannerTimings getPlannerTimings()
   {
      return plannerTimings;
   }

   public void setRequestId(int requestId)
   {
      this.requestId = requestId;
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

   public void setGoalPose(Pose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setException(Exception exception)
   {
      this.exception = exception;
   }

   public void setPacket(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      outputStatus.setPlanId(getRequestId());
      outputStatus.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(getFootstepPlan(), -1.0, -1.0));
      outputStatus.getBodyPath().clear();
      outputStatus.getGoalPose().set(getGoalPose());
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
