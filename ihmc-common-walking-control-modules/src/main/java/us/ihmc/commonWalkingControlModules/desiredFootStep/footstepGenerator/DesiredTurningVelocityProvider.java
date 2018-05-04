package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for obtaining the current desired walking turning velocity.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface DesiredTurningVelocityProvider
{
   /**
    * Gets the current desired turning velocity.
    * <p>
    * A positive value corresponds to turning to the left.
    * </p>
    * 
    * @return the desired turning velocity.
    */
   double getTurningVelocity();
}
