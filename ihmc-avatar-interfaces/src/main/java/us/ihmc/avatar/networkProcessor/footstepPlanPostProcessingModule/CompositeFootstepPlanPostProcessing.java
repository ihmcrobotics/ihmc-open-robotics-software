package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import us.ihmc.footstepPlanning.postProcessing.FootstepPlanPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.PostProcessingEnum;

import java.util.EnumMap;

/**
 * This class is meant to combine multiple instances of {@link FootstepPlanPostProcessingElement}, and then iteratively execute them.
 */
public class CompositeFootstepPlanPostProcessing implements FootstepPlanPostProcessingElement
{
   private final EnumMap<PostProcessingEnum, FootstepPlanPostProcessingElement> postProcessingElements = new EnumMap<>(PostProcessingEnum.class);

   public void addPostProcessingElement(FootstepPlanPostProcessingElement postProcessingElement)
   {
      if (postProcessingElements.containsKey(postProcessingElement.getElementName()))
         throw new RuntimeException("The composite builder already contains this element!");

      postProcessingElements.put(postProcessingElement.getElementName(), postProcessingElement);
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return true;
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPostProcessingPacket postProcessFootstepPlan(FootstepPostProcessingPacket outputStatus)
   {
      FootstepPostProcessingPacket currentOutputStatus = new FootstepPostProcessingPacket(outputStatus);
      for (FootstepPlanPostProcessingElement element : postProcessingElements.values())
      {
         if (!element.isActive())
            continue;

         currentOutputStatus = element.postProcessFootstepPlan(currentOutputStatus);
      }

      return currentOutputStatus;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.COMPOSITE;
   }
}
