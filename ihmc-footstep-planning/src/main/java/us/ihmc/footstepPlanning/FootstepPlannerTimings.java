package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningTimingsMessage;

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
      stepPlanningIterations = -1;
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
      setStepPlanningIterations(packet.getStepPlanningIterations());
   }

   public void setPacket(FootstepPlanningTimingsMessage packet)
   {
      packet.setTotalElapsedSeconds(getTotalElapsedSeconds());
      packet.setTimeBeforePlanningSeconds(getTimeBeforePlanningSeconds());
      packet.setTimePlanningBodyPathSeconds(getTimePlanningBodyPathSeconds());
      packet.setTimePlanningStepsSeconds(getTimePlanningStepsSeconds());
      packet.setStepPlanningIterations(getStepPlanningIterations());
   }
}
