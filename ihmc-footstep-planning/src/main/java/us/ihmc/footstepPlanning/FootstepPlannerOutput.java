package us.ihmc.footstepPlanning;

import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

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
    * Planned body path. Empty if planner failed
    */
   private final List<Pose3D> bodyPath = new ArrayList<>();

   /**
    * Planned body path before smoothing. Empty if planner failed
    */
   private final List<Point3D> bodyPathUnsmoothed = new ArrayList<>();

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

   /**
    * Swing trajectories, helpful for visualization
    */
   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories;

   public FootstepPlannerOutput()
   {
      clear();
   }

   /** Deep copy constructor */
   public FootstepPlannerOutput(FootstepPlannerOutput other)
   {
      requestId = other.requestId;
      footstepPlan.set(other.footstepPlan);
      bodyPathPlanningResult = other.bodyPathPlanningResult;
      footstepPlanningResult = other.footstepPlanningResult;
      for (Pose3D pose3D : other.bodyPath)
      {
         bodyPath.add(new Pose3D(pose3D));
      }
      for (Point3D point3D : other.bodyPathUnsmoothed)
      {
         bodyPathUnsmoothed.add(new Point3D(point3D));
      }
      goalPose.set(other.goalPose);
      if (other.exception != null)
         exception = new Exception(other.exception);
      plannerTimings.set(other.plannerTimings);
      if (other.swingTrajectories != null)
      {
         swingTrajectories = new ArrayList<>();
         for (EnumMap<Axis3D, List<PolynomialReadOnly>> otherSwingTrajectory : other.swingTrajectories)
         {
            if (otherSwingTrajectory != null)
            {
               EnumMap<Axis3D, List<PolynomialReadOnly>> swingTrajectory = new EnumMap<>(Axis3D.class);

               for (Map.Entry<Axis3D, List<PolynomialReadOnly>> axis3DListEntry : otherSwingTrajectory.entrySet())
               {
                  List<PolynomialReadOnly> polynomials = new ArrayList<>();
                  for (PolynomialReadOnly polynomialReadOnly : axis3DListEntry.getValue())
                  {
                     Polynomial polynomial = new Polynomial((Polynomial) polynomialReadOnly);
                     polynomials.add(polynomial);
                  }
                  swingTrajectory.put(axis3DListEntry.getKey(), polynomials);
               }

               swingTrajectories.add(swingTrajectory);
            }
            else
            {
               swingTrajectories.add(null);
            }
         }
      }
   }

   public void clear()
   {
      requestId = -1;
      footstepPlan.clear();
      bodyPathPlanningResult = null;
      footstepPlanningResult = null;
      bodyPath.clear();
      bodyPathUnsmoothed.clear();
      goalPose.setToNaN();
      exception = null;
      plannerTimings.clear();
      swingTrajectories = null;
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

   public List<EnumMap<Axis3D, List<PolynomialReadOnly>>> getSwingTrajectories()
   {
      return swingTrajectories;
   }

   public List<Pose3D> getBodyPath()
   {
      return bodyPath;
   }

   public List<Point3D> getBodyPathUnsmoothed()
   {
      return bodyPathUnsmoothed;
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

   public void setGoalPose(Pose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setException(Exception exception)
   {
      this.exception = exception;
   }

   public void setSwingTrajectories(List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories)
   {
      this.swingTrajectories = swingTrajectories;
   }

   public void setPacket(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      outputStatus.setPlanId(getRequestId());
      outputStatus.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(getFootstepPlan(), -1.0, -1.0));
      outputStatus.getBodyPath().clear();
      outputStatus.getBodyPathUnsmoothed().clear();
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

      for (int i = 0; i < bodyPath.size(); i++)
      {
         outputStatus.getBodyPath().add().set(bodyPath.get(i));
      }

      for (int i = 0; i < bodyPathUnsmoothed.size(); i++)
      {
         outputStatus.getBodyPathUnsmoothed().add().set(bodyPathUnsmoothed.get(i));
      }
   }
}
