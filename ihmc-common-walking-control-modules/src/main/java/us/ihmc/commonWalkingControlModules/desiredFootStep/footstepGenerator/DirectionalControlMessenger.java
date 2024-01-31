package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for requesting the desired velocity for the controller
 * </p>
 *
 * @author Robert Griffin
 */
public interface DirectionalControlMessenger
{
   /**
    * Submits the request to the controller.
    */
   void submitDirectionalControlRequest(double desiredXVelocity, double desiredYVelocity, double desiredTurningSpeed);

   void submitGaitParameters(double swingHeight, double swingDuration, double doubleSupportFraction);
}
