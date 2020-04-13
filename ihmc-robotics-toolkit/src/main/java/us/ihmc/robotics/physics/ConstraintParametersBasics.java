package us.ihmc.robotics.physics;

/**
 * Write and read interface for accessing/modifying a set of parameters used for resolving general
 * constraints in {@link ExperimentalPhysicsEngine}.
 * <p>
 * Constraints can either be: robot joint limits or contact between two collidables.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface ConstraintParametersBasics extends ConstraintParametersReadOnly
{
   /**
    * Performs deep copy of {@code other} into {@code this}.
    * 
    * @param other the other set of parameters. Not modified.
    */
   default void set(ConstraintParametersReadOnly other)
   {
      setCoefficientOfRestitution(other.getCoefficientOfRestitution());
      setConstraintForceMixing(other.getConstraintForceMixing());
      setErrorReductionParameter(other.getErrorReductionParameter());
   }

   /**
    * Sets the coefficient of restitution.
    * <p>
    * When resolving a constraint with an impulse, the coefficient of restitution defines the relative
    * velocity post-impulse, see
    * <a href="https://en.wikipedia.org/wiki/Coefficient_of_restitution">Wikipedia article</a>.
    * </p>
    * <p>
    * The coefficient of restitution, <i>e</i> below, is recommended to be in [0, 1]:
    * <ul>
    * <li><i>e</i> = 0: The constraint will be perfectly inelastic, the relative velocity post-impulse
    * along the collision axis is 0.
    * <li><i>e</i> = 1: The constraint will be perfectly elastic, the relative velocity post-impulse
    * along the collision axis is equal to the relative pre-impulse negated: the objects interacting
    * perfectly rebound with respect to each other.
    * <li><i>e</i> &in; ]0, 1[: "real-world" inelastic collision, where some of the kinetic energy is
    * dissipated.
    * </ul>
    * </p>
    * 
    * @param coefficientOfRestitution the coefficient of restitution, recommended [0, 0.1].
    */
   void setCoefficientOfRestitution(double coefficientOfRestitution);

   /**
    * Set the constraint force mixing parameter.
    * <p>
    * This parameter is inspired on the homonym in the Open Dynamics Engine, see
    * <a href="https://ode.org/ode-latest-userguide.html#sec_3_8"> ODE user guide</a>.
    * </p>
    * <p>
    * The constraint force mixing, or CFM, essentially reduces the effectiveness of the impulse for
    * resolving a constraint, which in turns allows violation of the constraint. It is recommended to
    * use values in [0, 1]:
    * <ul>
    * <li>CFM = 0: constraints are resolved but the resulting impulses are completely cancelled making
    * it appear that constraints are not resolved.
    * <li>CFM = 1 (recommended): constraints are resolved and the computed impulses are applied at
    * 100%, observed violation of the constraints should be minimum to none.
    * <li>CFM = [0, 1]: constraints are resolved and only a part of the impulses are applied. The
    * constraints are violated.
    * </ul>
    * </p>
    * 
    * @param constraintForceMixing the constraint force mixing, recommended 1.
    */
   void setConstraintForceMixing(double constraintForceMixing);

   /**
    * Sets the error reduction parameter.
    * <p>
    * This parameter is inspired on the homonym in the Open Dynamics Engine, see
    * <a href="https://ode.org/ode-latest-userguide.html#sec_3_7"> ODE user guide</a>.
    * </p>
    * <p>
    * The error reduction parameter, or ERP, indicates the percentage of error in the constraint that
    * should be resolved in each simulation tick. The parameter is defined in [0, 1]:
    * <ul>
    * <li>ERP = 0: no correction is applied, if there is constraint error no special effort will be
    * added to resolve it such that it can only stabilize or grow.
    * <li>ERP = 1: a correction is applied to correct the integrity of the constraint error in a single
    * tick. This is expected to provide an unstable simulation.
    * <li>ERP &in; [0, 1]: a correction is applied to correct a percentage of the constraint error in a
    * single tick.
    * </ul>
    * </p>
    * 
    * @param errorReductionParameter the error reduction parameter, recommended [0, 0.2].
    */
   void setErrorReductionParameter(double errorReductionParameter);
}
