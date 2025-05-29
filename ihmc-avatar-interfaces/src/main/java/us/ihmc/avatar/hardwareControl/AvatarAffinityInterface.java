package us.ihmc.avatar.hardwareControl;

import us.ihmc.affinity.Processor;
import us.ihmc.realtime.PriorityParameters;

public interface AvatarAffinityInterface
{
   Processor getSchedulerThreadProcessor();

   Processor getEstimatorThreadProcessor();

   Processor getControlThreadProcessor();

   Processor getStepGeneratorThreadProcessor();

   PriorityParameters getSchedulerPriority();

   PriorityParameters getEstimatorPriority();

   PriorityParameters getControllerPriority();

   PriorityParameters getStepGeneratorPriority();
}
