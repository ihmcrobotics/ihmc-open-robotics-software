package us.ihmc.aware.controller.force.taskSpaceController;

public interface QuadrupedTaskSpaceFilterBlock
{
   void reset();
   void compute(QuadrupedTaskSpaceCommands commands);
}
