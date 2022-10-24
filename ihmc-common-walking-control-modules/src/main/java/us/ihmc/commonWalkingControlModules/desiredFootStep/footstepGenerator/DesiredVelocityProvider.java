package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * This interface is part of the API of {@link ContinuousStepGenerator}.
 * <p>
 * It is used to provide a protocol for obtaining the current desired walking forward/lateral
 * velocity.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface DesiredVelocityProvider
{
   /**
    * Gets the current desired forward/lateral velocity.
    * <p>
    * A positive x value corresponds to walking forward, a positive y value corresponds to walking to
    * the left.
    * </p>
    * 
    * @return the desired forward/lateral velocity.
    */
   Vector2DReadOnly getDesiredVelocity();

   /**
    * Whether this velocity provider provides unit velocity or actual velocity in [m/s].
    * <ul>
    * <li>if {@code false}, it is assumed that {@link #getDesiredVelocity()} provides a desired
    * velocity in [m/s].
    * <li>if {@code true}, it is assumed that {@link #getDesiredVelocity()} provides a desired velocity
    * contained in [-1, 1], which is then to be scaled by the min/max velocity achievable given the
    * stepping parameters. For instance, along the x-axis, the maximum velocity achievable is defined
    * by the maximum step length and the step duration.
    * </ul>
    * 
    * @return whether this provider uses unit velocity or not. Default value is {@code false}.
    */
   default boolean isUnitVelocity()
   {
      return false;
   }
}
