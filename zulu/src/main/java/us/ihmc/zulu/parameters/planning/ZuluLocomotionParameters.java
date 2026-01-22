package us.ihmc.zulu.parameters.planning;

import us.ihmc.footstepPlanning.LocomotionParameters;

public class ZuluLocomotionParameters extends LocomotionParameters
{
   public ZuluLocomotionParameters()
   {
      super(ZuluLocomotionParameters.class);
      loadUnsafe();
   }
}
