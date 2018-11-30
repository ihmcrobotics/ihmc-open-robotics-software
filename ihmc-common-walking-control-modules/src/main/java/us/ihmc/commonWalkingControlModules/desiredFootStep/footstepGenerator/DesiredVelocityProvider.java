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
    * A positive x value corresponds to walking forward, a positive y value corresponds to walking
    * to the left.
    * </p>
    * 
    * @return the desired forward/lateral velocity.
    */
   Vector2DReadOnly getDesiredVelocity();
}
