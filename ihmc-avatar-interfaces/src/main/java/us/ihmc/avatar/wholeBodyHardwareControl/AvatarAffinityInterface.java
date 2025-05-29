package us.ihmc.avatar.wholeBodyHardwareControl;

import us.ihmc.affinity.Processor;
import us.ihmc.realtime.PriorityParameters;

/**
 * This class is used to define thread processor and priority assignment
 * for realtime control on hardware
 *
 * @author Stefan Fasano
 */
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
