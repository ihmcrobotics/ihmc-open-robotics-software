package us.ihmc.aware.controller.force.taskSpaceController;

public interface QuadrupedTaskSpaceControlBlock
{
   void reset();
   void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands);
}
