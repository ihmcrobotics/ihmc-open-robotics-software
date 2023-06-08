package us.ihmc.footstepPlanning;

import toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage;

public class FootstepPlannerTimings
{
   /**
    * Total time measured in the planner process between receiving request message and publishing output message
    */
   private double totalElapsedSeconds;

   /**
    * Elapsed time between receiving request message and starting to plan body path
    */
   private double timeBeforePlanningSeconds;

   /**
    * Elapsed time for planning body path
    */
   private double timePlanningBodyPathSeconds;

   /**
    * Elapsed time for step planning
    */
   private double timePlanningStepsSeconds;

   /**
    * Number of iterations performed path planning
    */
   private long pathPlanningIterations;

   /**
    * Number of iterations performed during step planning
    */
   private long stepPlanningIterations;

   public FootstepPlannerTimings()
   {
      clear();
   }

   public void clear()
   {
      totalElapsedSeconds = -1.0;
      timeBeforePlanningSeconds = -1.0;
      timePlanningBodyPathSeconds = -1.0;
      timePlanningStepsSeconds = -1.0;
      pathPlanningIterations = -1;
      stepPlanningIterations = -1;
   }

   public void set(FootstepPlannerTimings other)
   {
      totalElapsedSeconds = other.totalElapsedSeconds;
      timeBeforePlanningSeconds = other.timeBeforePlanningSeconds;
      timePlanningBodyPathSeconds = other.timePlanningBodyPathSeconds;
      timePlanningStepsSeconds = other.timePlanningStepsSeconds;
      pathPlanningIterations = other.pathPlanningIterations;
      stepPlanningIterations = other.stepPlanningIterations;
   }

   public void setTotalElapsedSeconds(double totalElapsedSeconds)
   {
      this.totalElapsedSeconds = totalElapsedSeconds;
   }

   public void setTimeBeforePlanningSeconds(double timeBeforePlanningSeconds)
   {
      this.timeBeforePlanningSeconds = timeBeforePlanningSeconds;
   }

   public void setTimePlanningBodyPathSeconds(double timePlanningBodyPathSeconds)
   {
      this.timePlanningBodyPathSeconds = timePlanningBodyPathSeconds;
   }

   public void setTimePlanningStepsSeconds(double timePlanningStepsSeconds)
   {
      this.timePlanningStepsSeconds = timePlanningStepsSeconds;
   }

   public void setPathPlanningIterations(long pathPlanningIterations)
   {
      this.pathPlanningIterations = pathPlanningIterations;
   }

   public void setStepPlanningIterations(long stepPlanningIterations)
   {
      this.stepPlanningIterations = stepPlanningIterations;
   }

   public double getTotalElapsedSeconds()
   {
      return totalElapsedSeconds;
   }

   public double getTimeBeforePlanningSeconds()
   {
      return timeBeforePlanningSeconds;
   }

   public double getTimePlanningBodyPathSeconds()
   {
      return timePlanningBodyPathSeconds;
   }

   public double getTimePlanningStepsSeconds()
   {
      return timePlanningStepsSeconds;
   }

   public long getPathPlanningIterations()
   {
      return pathPlanningIterations;
   }

   public long getStepPlanningIterations()
   {
      return stepPlanningIterations;
   }

   public void setFromPacket(FootstepPlanningTimingsMessage packet)
   {
      setTotalElapsedSeconds(packet.getTotalElapsedSeconds());
      setTimeBeforePlanningSeconds(packet.getTimeBeforePlanningSeconds());
      setTimePlanningBodyPathSeconds(packet.getTimePlanningBodyPathSeconds());
      setTimePlanningStepsSeconds(packet.getTimePlanningStepsSeconds());
      setPathPlanningIterations(packet.getPathPlanningIterations());
      setStepPlanningIterations(packet.getStepPlanningIterations());
   }

   public void setPacket(FootstepPlanningTimingsMessage packet)
   {
      packet.setTotalElapsedSeconds(getTotalElapsedSeconds());
      packet.setTimeBeforePlanningSeconds(getTimeBeforePlanningSeconds());
      packet.setTimePlanningBodyPathSeconds(getTimePlanningBodyPathSeconds());
      packet.setTimePlanningStepsSeconds(getTimePlanningStepsSeconds());
      packet.setPathPlanningIterations(getPathPlanningIterations());
      packet.setStepPlanningIterations(getStepPlanningIterations());
   }
}
