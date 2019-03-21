package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class VirtualModelControlOptimizationSettingsCommand implements VirtualModelControlCommand<VirtualModelControlOptimizationSettingsCommand>
{
   private double rhoMin = Double.NaN;
   private double rhoWeight = Double.NaN;
   private double rhoRateWeight = Double.NaN;
   private final Vector2D centerOfPressureWeight = new Vector2D(Double.NaN, Double.NaN);
   private final Vector2D centerOfPressureRateWeight = new Vector2D(Double.NaN, Double.NaN);
   private double momentumRateWeight = Double.NaN;
   private double momentumAccelerationWeight = Double.NaN;

   /**
    * Sets the minimum force value to apply at each basis vector of each contact point.
    * <p>
    * A positive value will ensure that the optimization is setup to satisfy a unilateral contact
    * constraint for each contacting body. A non-zero and positive value ensures that any rigid-body
    * that is assumed to be in contact will exert a minimum amount of force onto the environment. This
    * tends to reduce foot slipping.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @param rhoMin the minimum value for each component of rho.
    */
   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
   }

   /**
    * Sets the weight specifying how much high contact force values should be penalized in the
    * optimization problem.
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably be above {@code 1.0e-8}. A high value will cause the system
    * to become too 'floppy'. A value of {@code 1.0e-5} is used for Atlas' simulations. Be careful as
    * the weight is for forces and thus is affected by the total weight of the robot. A higher value
    * should be picked for a lighter robot.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @param rhoWeight the weight to use for contact force regularization.
    */
   public void setRhoWeight(double rhoWeight)
   {
      this.rhoWeight = rhoWeight;
   }

   /**
    * Sets the default weight specifying how much high variations of contact forces should be penalized
    * in the optimization problem.
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight helps
    * to improve smoothness of the resulting forces. A high value will cause the system to become too
    * 'floppy' and unresponsive. A value of {@code 3.2e-8} is used for Atlas' simulations. Be careful
    * as the weight is for forces and thus is affected by the total weight of the robot. A higher value
    * should be picked for a lighter robot.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @param rhoRateWeight the weight to use for the regularization of the rate of change of contact
    *           forces.
    */
   public void setRhoRateWeight(double rhoRateWeight)
   {
      this.rhoRateWeight = rhoRateWeight;
   }

   /**
    * Sets the weight specifying how much deviation of the desired center of pressure (CoP) off of the
    * contact support centroid should be penalized.
    * <p>
    * In other words, a high positive value will result in the controller core trying to keep the CoP
    * in the middle of each foot for instance. This value does not need to be non-zero. A value of
    * about {@code 0.001} is used for Atlas' simulations.
    * </p>
    *
    * @param centerOfPressureWeight the regularization weight to use on the center of pressure
    *           location.
    */
   public void setCenterOfPressureWeight(Tuple2DReadOnly centerOfPressureWeight)
   {
      this.centerOfPressureWeight.set(centerOfPressureWeight);
   }

   /**
    * Sets the weight specifying how much variations of the desired center of pressure should be
    * penalized.
    * <p>
    * The value should be positive but not necessarily non-zero. It helps smoothing the motion of the
    * center pressure of each contacting body. A value of about {@code 3.2e-8} is used for Atlas'
    * simulations.
    * </p>
    *
    * @param centerOfPressureRateWeight the regularization weight to use for center of pressure
    *           variations.
    */
   public void setCenterOfPressureRateWeight(Tuple2DReadOnly centerOfPressureRateWeight)
   {
      this.centerOfPressureRateWeight.set(centerOfPressureRateWeight);
   }

   /**
    * Sets the weight specifying how much high values for the rate of change of momentum should be
    * penalized.
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably by above {@code 1.0e-8}.
    * </p>
    * 
    * @param momentumRateWeight the weight to use for rate of change of momentum regularization.
    */
   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;
   }

   /**
    * Sets the weight specifying how much high values for the momentum acceleration should be
    * penalized.
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weights helps
    * to improve smoothness of the resulting motions.
    * </p>
    * 
    * @param momentumAccelerationWeight the weight to use for rate of change of momentum
    *           regularization.
    */
   public void setMomentumAccelerationWeight(double momentumAccelerationWeight)
   {
      this.momentumAccelerationWeight = momentumAccelerationWeight;
   }

   /**
    * Whether this command holds onto a new value for {@code rhoMin} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasRhoMin()
   {
      return !Double.isNaN(rhoMin);
   }

   /**
    * Whether this command holds onto a new value for {@code rhoWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasRhoWeight()
   {
      return !Double.isNaN(rhoWeight);
   }

   /**
    * Whether this command holds onto a new value for {@code rhoRateWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasRhoRateWeight()
   {
      return !Double.isNaN(rhoRateWeight);
   }

   /**
    * Whether this command holds onto a new value for {@code centerOfPressureWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasCenterOfPressureWeight()
   {
      return !centerOfPressureWeight.containsNaN();
   }

   /**
    * Whether this command holds onto a new value for {@code centerOfPressureRateWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasCenterOfPressureRateWeight()
   {
      return !centerOfPressureRateWeight.containsNaN();
   }

   /**
    * Whether this command holds onto a new value for {@code momentumRateWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasMomentumRateWeight()
   {
      return !Double.isNaN(momentumRateWeight);
   }

   /**
    * Whether this command holds onto a new value for {@code momentumAccelerationWeight} or not.
    * 
    * @return {@code true} if this command carries an actual value for this field.
    */
   public boolean hasMomentumAccelerationWeight()
   {
      return !Double.isNaN(momentumAccelerationWeight);
   }

   /**
    * Gets the value for {@code rhoMin}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code rhoMin}.
    * @see #hasRhoMin()
    * @see #setRhoMin(double)
    */
   public double getRhoMin()
   {
      return rhoMin;
   }

   /**
    * Gets the value for {@code rhoWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code rhoWeight}.
    * @see #hasRhoWeight()
    * @see #setRhoWeight(double)
    */
   public double getRhoWeight()
   {
      return rhoWeight;
   }

   /**
    * Gets the value for {@code rhoRateWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code rhoRateWeight}.
    * @see #hasRhoRateWeight()
    * @see #setRhoRateWeight(double)
    */
   public double getRhoRateWeight()
   {
      return rhoRateWeight;
   }

   /**
    * Gets the value for {@code centerOfPressureWeight}.
    * <p>
    * It is set to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code centerOfPressureWeight}.
    * @see #hasCenterOfPressureWeight()
    * @see #setCenterOfPressureWeight(Tuple2DReadOnly)
    */
   public Vector2D getCenterOfPressureWeight()
   {
      return centerOfPressureWeight;
   }

   /**
    * Gets the value for {@code centerOfPressureRateWeight}.
    * <p>
    * It is set to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code centerOfPressureRateWeight}.
    * @see #hasCenterOfPressureRateWeight()
    * @see #setCenterOfPressureRateWeight(Tuple2DReadOnly)
    */
   public Vector2D getCenterOfPressureRateWeight()
   {
      return centerOfPressureRateWeight;
   }

   /**
    * Gets the value for {@code momentumRateWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code momentumRateWeight}.
    * @see #hasMomentumRateWeight()
    * @see #setMomentumRateWeight(double)
    */
   public double getMomentumRateWeight()
   {
      return momentumRateWeight;
   }

   /**
    * Gets the value for {@code momentumAccelerationWeight}.
    * <p>
    * It is equal to {@code Double#NaN} if this command does not hold onto a new value for this field.
    * </p>
    * 
    * @return the new value for {@code momentumAccelerationWeight}.
    * @see #hasMomentumAccelerationWeight()
    * @see #setMomentumAccelerationWeight(double)
    */
   public double getMomentumAccelerationWeight()
   {
      return momentumAccelerationWeight;
   }

   @Override
   public void set(VirtualModelControlOptimizationSettingsCommand other)
   {
      rhoMin = other.rhoMin;
      rhoWeight = other.rhoWeight;
      rhoRateWeight = other.rhoRateWeight;
      centerOfPressureWeight.set(other.centerOfPressureWeight);
      centerOfPressureRateWeight.set(other.centerOfPressureRateWeight);
      momentumRateWeight = other.momentumRateWeight;
      momentumAccelerationWeight = other.momentumAccelerationWeight;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof VirtualModelControlOptimizationSettingsCommand)
      {
         VirtualModelControlOptimizationSettingsCommand other = (VirtualModelControlOptimizationSettingsCommand) object;
         if (Double.compare(rhoMin, other.rhoMin) != 0)
            return false;
         if (Double.compare(rhoWeight, other.rhoWeight) != 0)
            return false;
         if (Double.compare(rhoRateWeight, other.rhoRateWeight) != 0)
            return false;
         if (centerOfPressureWeight.containsNaN() ^ other.centerOfPressureWeight.containsNaN())
            return false;
         if (!centerOfPressureWeight.containsNaN() && !centerOfPressureWeight.equals(other.centerOfPressureWeight))
            return false;
         if (centerOfPressureRateWeight.containsNaN() ^ other.centerOfPressureRateWeight.containsNaN())
            return false;
         if (!centerOfPressureRateWeight.containsNaN() && !centerOfPressureRateWeight.equals(other.centerOfPressureRateWeight))
            return false;
         if (Double.compare(momentumRateWeight, other.momentumRateWeight) != 0)
            return false;
         if (Double.compare(momentumAccelerationWeight, other.momentumAccelerationWeight) != 0)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": rho min: " + rhoMin + ", rho weight: " + rhoWeight + ", rho rate weight: " + rhoRateWeight + ", CoP weight: "
            + centerOfPressureWeight + ", CoP rate weight: " + centerOfPressureRateWeight + ", momentum rate weight: " + momentumRateWeight
            + ", momentum acceleration weight: " + momentumAccelerationWeight;
   }
}
