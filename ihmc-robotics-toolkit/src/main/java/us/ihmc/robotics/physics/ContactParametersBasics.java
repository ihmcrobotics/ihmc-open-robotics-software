package us.ihmc.robotics.physics;

/**
 * Write and read interface for accessing/modifying a set of parameters used for resolving contacts
 * between pairs of collidables, see {@link ExperimentalPhysicsEngine}.
 * 
 * @author Sylvain Bertrand
 */
public interface ContactParametersBasics extends ContactParametersReadOnly, ConstraintParametersBasics
{
   default void set(ContactParametersReadOnly other)
   {
      ConstraintParametersBasics.super.set(other);
      setMinimumPenetration(other.getMinimumPenetration());
      setCoefficientOfFriction(other.getCoefficientOfFriction());
      setSlipErrorReductionParameter(other.getSlipErrorReductionParameter());
   }

   /**
    * Sets the minimum distance by which two collidable should be penetrating each other before
    * resolving the contact.
    * <p>
    * Ideally the contact should be resolved when the collidables are touching. However, when only
    * touching, it is impossible to estimate the contact normal which is essential to solving the
    * problem. By letting the collidables penetrate a little, this allows to estimate the contact
    * normal. A larger minimum penetration implies greater robustness to numerical errors when
    * estimating the normal.
    * </p>
    * 
    * @param minimumPenetration the distance before resolving the contact, recommended ~1.0e-5.
    */
   void setMinimumPenetration(double minimumPenetration);

   /**
    * Sets the coefficient of friction.
    * <p>
    * Assuming the Coulomb friction model is used, the coefficient of friction <tt>&mu;</tt> defines
    * the relationship between normal contact force <tt>F<sub>n</sub></tt> and maximum achievable
    * tangential contact force <tt>F<sub>t</sub></tt>:
    * 
    * <pre>
    * |F<sub>t</sub>| &leq; &mu; F<sub>n</sub>
    * </pre>
    * </p>
    * <p>
    * The value of the coefficient of friction is commonly in [0, 1]:
    * <ul>
    * <li><tt>&mu;</tt> = 0: frictionless contacts, zero resistance to sliding is added when resolving
    * contact between two collidables.
    * <li><tt>&mu;</tt> &in; [0, 1]: "real-world" friction, the amount of the resistance to sliding is
    * non-negligible but still limited.
    * </ul>
    * </p>
    * <p>
    * Note that the same coefficient is used for both static and sliding contacts.
    * </p>
    * 
    * @param coefficientOfFriction the coefficient of friction, recommended 0.7.
    */
   void setCoefficientOfFriction(double coefficientOfFriction);

   /**
    * Sets the value of the error reduction parameter used to reduce slip.
    * <p>
    * Like the ERP, this parameter indicates the percentage of error in the contact that should be
    * resolved in each simulation tick. This parameter is defined in [0, 1]:
    * <ul>
    * <li>ERP = 0: no correction is applied, if there is constraint error no special effort will be
    * added to resolve it such that it can only stabilize or grow.
    * <li>ERP = 1: a correction is applied to correct the integrity of the constraint error in a single
    * tick. This is expected to provide an unstable simulation.
    * <li>ERP &in; [0, 1]: a correction is applied to correct a percentage of the constraint error in a
    * single tick.
    * </ul>
    * </p>
    * <p>
    * While contact impulses are resolved such as there is no slip occurring, numerical error and other
    * artifacts of the kind can impair the ability of the solver to perfectly counter slip. As a
    * result, 2 shapes may slowly over-time slip with respect to each other.
    * </p>
    * 
    * @param slipErrorReductionParameter the error reduction parameter to use for slip.
    */
   void setSlipErrorReductionParameter(double slipErrorReductionParameter);
}
