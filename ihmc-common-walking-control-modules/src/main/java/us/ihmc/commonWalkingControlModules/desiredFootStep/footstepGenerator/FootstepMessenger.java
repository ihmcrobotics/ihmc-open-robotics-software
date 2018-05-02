package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.FootstepDataListMessage;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for sending generated footsteps to the controller.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FootstepMessenger
{
   /**
    * Submits the generated footsteps to the controller.
    * 
    * @param footsteps the generated footsteps from {@code ContinuousStepGenerator}.
    */
   void submitFootsteps(FootstepDataListMessage footsteps);
}
