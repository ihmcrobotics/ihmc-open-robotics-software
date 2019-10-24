package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;

public interface FootstepPlanPostProcessingElement
{
   /**
    * Checks whether or not this post processing element should be used.
    */
   boolean isActive();

   void setIsActive(boolean isActive);

   /**
    * Takes the output status of an element, and post processes it using the planar region list.
    * @param outputStatus raw output status
    * @return processed output status
    */
   FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningToolboxOutputStatus outputStatus);

   /**
    * Gets the name of the current post processing element
    */
   PostProcessingEnum getElementName();
}
