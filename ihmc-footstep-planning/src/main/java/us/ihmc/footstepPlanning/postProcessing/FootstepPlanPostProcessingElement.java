package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepPostProcessingPacket;

public interface FootstepPlanPostProcessingElement
{
   /**
    * Checks whether or not this post processing element should be used.
    */
   boolean isActive();

   /**
    * Takes the output status of an element, and post processes it using the planar region list.
    * @param planToProcess raw plan
    * @return processed plan
    */
   FootstepPostProcessingPacket postProcessFootstepPlan(FootstepPostProcessingPacket planToProcess);

   /**
    * Gets the name of the current post processing element
    */
   PostProcessingEnum getElementName();
}
