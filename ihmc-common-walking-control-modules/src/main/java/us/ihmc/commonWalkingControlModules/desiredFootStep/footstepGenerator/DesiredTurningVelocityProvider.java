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

   /**
    * Whether this velocity provider provides unit velocity or actual velocity in [m/s].
    * <ul>
    * <li>if {@code false}, it is assumed that {@link #getTurningVelocity()} provides a desired
    * velocity in [m/s].
    * <li>if {@code true}, it is assumed that {@link #getTurningVelocity()} provides a desired velocity
    * contained in [-1, 1], which is then to be scaled by the min/max velocity achievable given the
    * stepping parameters.
    * </ul>
    * 
    * @return whether this provider uses unit velocity or not. Default value is {@code false}.
    */
   default boolean isUnitVelocity()
   {
      return false;
   }
}
