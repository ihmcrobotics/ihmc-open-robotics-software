package us.ihmc.openAlexander.parameters.planning;

import us.ihmc.footstepPlanning.LocomotionParameters;

public class AlexanderLocomotionParameters extends LocomotionParameters
{
   public AlexanderLocomotionParameters()
   {
      super(AlexanderLocomotionParameters.class);
      loadUnsafe();
   }
}
