package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for requesting the controller to stop walking.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface StopWalkingMessenger
{
   /**
    * Submits the request to the controller.
    */
   void submitStopWalkingRequest();
}
