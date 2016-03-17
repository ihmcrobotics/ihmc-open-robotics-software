package us.ihmc.aware.controller.force.taskSpaceController;

public interface QuadrupedTaskSpaceCommandFilter
{
   void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands);
}
