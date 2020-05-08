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
      setCoefficientOfFriction(other.getCoefficientOfFriction());
   }

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
}
