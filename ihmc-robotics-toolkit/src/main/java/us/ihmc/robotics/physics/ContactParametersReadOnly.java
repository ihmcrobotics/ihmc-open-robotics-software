package us.ihmc.robotics.physics;

/**
 * Read-only interface for accessing a set of parameters used for resolving contacts between pairs
 * of collidables, see {@link ExperimentalPhysicsEngine}.
 * 
 * @author Sylvain Bertrand
 */
public interface ContactParametersReadOnly extends ConstraintParametersReadOnly
{
   /**
    * Returns the minimum distance by which two collidable should be penetrating each other before
    * resolving the contact.
    * <p>
    * Ideally the contact should be resolved when the collidables are touching. However, when only
    * touching, it is impossible to estimate the contact normal which is essential to solving the
    * problem. By letting the collidables penetrate a little, this allows to estimate the contact
    * normal. A larger minimum penetration implies greater robustness to numerical errors when
    * estimating the normal.
    * </p>
    * 
    * @return the minimum penetration distance before resolving the contact.
    */
   double getMinimumPenetration();

   /**
    * Returns the value of the coefficient of friction.
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
    * @return the coefficient of friction.
    */
   double getCoefficientOfFriction();

   /**
    * Returns whether a moment-impulse of friction should be calculated alongside the usual linear
    * impulse.
    * <p>
    * When enabled, only a moment around the normal axis of an active contact is computed with the goal
    * of canceling the angular velocity around the normal axis.
    * </p>
    * 
    * @return {@code true} to enable the friction moment calculation, {@code false} to disable it.
    */
   boolean getComputeFrictionMoment();

   /**
    * When computing the moment-impulse of friction for a contact, then Coulomb friction is replaced by
    * an elliptic law as follows:
    * 
    * <pre>
    * F<sub>x</sub><sup>2</sup>/e<sub>x</sub><sup>2</sup> + F<sub>y</sub><sup>2</sup>/e<sub>y</sub><sup>2</sup> + T<sub>z</sub><sup>2</sup>/e<sub>zz</sub><sup>2</sup> &leq; &mu;F<sub>z</sub><sup>2</sup>
    * </pre>
    * 
    * where <tt>F<sub>x</sub></tt> and <tt>F<sub>y</sub></tt> are the tangential forces,
    * <tt>T<sub>z</sub></tt> the normal moment, <tt>F<sub>z</sub></tt> the normal force, and
    * <tt>&mu;</tt> the coefficient of friction. <tt>e<sub>i</sub> are positive constants defined by
    * the user.
    * <p>
    * In the current implementation of the solver, <tt>e<sub>x</sub> = e<sub>y</sub> = 1<tt> and only <tt>e<sub>zz</sub></tt>
    * is the ratio returned by this method.
    * </p>
    * 
    * @return the value to use for <tt>e<sub>zz</sub></tt>, default is {@code 0.3}.
    */
   double getCoulombMomentFrictionRatio();
}
