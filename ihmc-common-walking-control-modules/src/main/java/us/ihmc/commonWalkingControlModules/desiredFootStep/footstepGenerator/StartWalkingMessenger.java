package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for requesting the controller to start walking.
 * </p>
 * 
 * @author Robert Griffin
 */
public interface StartWalkingMessenger
{
   /**
    * Submits the request to the controller.
    */
   void submitStartWalkingRequest();
}
