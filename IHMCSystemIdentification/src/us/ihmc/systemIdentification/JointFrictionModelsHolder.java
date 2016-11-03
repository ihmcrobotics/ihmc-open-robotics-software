package us.ihmc.systemIdentification;

import java.util.EnumMap;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.systemIdentification.frictionId.frictionModels.FrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.FrictionState;
import us.ihmc.systemIdentification.frictionId.frictionModels.JointFrictionModel;


public abstract class JointFrictionModelsHolder
{
   private static final double SMALL_VELOCITY_ABS = 0.005;
   private static final double ACCELERATION_THRESHOLD = 0.1;
   private static final double ALPHA_EQUIVALENT_VELOCITY = 0.5;
   
   private final DoubleYoVariable stictionTransitionVelocity;
   private final AlphaFilteredYoVariable filteredVelocity;
   private final AlphaFilteredYoVariable velocityForFrictionCalculation;
   private final DoubleYoVariable alphaForFilteredVelocity;
   private final DoubleYoVariable forceThreshold;
   private final DoubleYoVariable frictionCompensationEffectiveness;
   private final DoubleYoVariable maxJointVelocityToCompensate;
   private final DoubleYoVariable smallVelocityAbs;
   private final DoubleYoVariable accelerationThreshold;

   protected final DoubleYoVariable frictionForce;
   protected final EnumYoVariable<FrictionState> frictionCompensationState;
   protected final EnumYoVariable<FrictionModel> activeFrictionModel;
   protected final EnumMap<FrictionModel, JointFrictionModel> frictionModels;

   public JointFrictionModelsHolder(String name, YoVariableRegistry registry, double alpha, double forceThreshold, double stictionTransitionVelocity,
         double maxJointVelocityToCompensate)
   {
      alphaForFilteredVelocity = new DoubleYoVariable(name + "_alphaForFilteredVelocity", registry);
      alphaForFilteredVelocity.set(alpha);
      frictionModels = new EnumMap<FrictionModel, JointFrictionModel>(FrictionModel.class);
      frictionCompensationState = new EnumYoVariable<FrictionState>(name + "_frictionCompensationState", registry, FrictionState.class);
      activeFrictionModel = new EnumYoVariable<FrictionModel>(name + "_activeFrictionModel", registry, FrictionModel.class);
      frictionForce = new DoubleYoVariable(name + "_frictionForce", registry);
      this.stictionTransitionVelocity = new DoubleYoVariable(name + "_stictionTransitionVelocity", registry);
      this.stictionTransitionVelocity.set(Math.abs(stictionTransitionVelocity));
      this.forceThreshold = new DoubleYoVariable(name + "_forceThreshold", registry);
      this.forceThreshold.set(Math.abs(forceThreshold));
      filteredVelocity = new AlphaFilteredYoVariable(name + "_alphaFilteredVelocity", registry, alphaForFilteredVelocity);
      filteredVelocity.update(0.0);
      velocityForFrictionCalculation = new AlphaFilteredYoVariable(name + "_velocityForFrictionCalculation", registry, ALPHA_EQUIVALENT_VELOCITY);
      velocityForFrictionCalculation.update(0.0);
      frictionCompensationEffectiveness = new DoubleYoVariable(name + "_frictionCompensationEffectiveness", registry);
      frictionCompensationEffectiveness.set(0.0);
      this.maxJointVelocityToCompensate = new DoubleYoVariable(name + "_maxJointVelocityToCompensate", registry);
      this.maxJointVelocityToCompensate.set(Math.abs(maxJointVelocityToCompensate));
      smallVelocityAbs = new DoubleYoVariable(name + "_stictionEquivalentVelocity", registry);
      smallVelocityAbs.set(Math.abs(SMALL_VELOCITY_ABS));
      accelerationThreshold = new DoubleYoVariable(name + "_accelerationThreshold", registry);
      this.accelerationThreshold.set(Math.abs(ACCELERATION_THRESHOLD)); // maybe take as parameter in the constructor and set in the friction parameters
   }

   /**
    * This method computes an equivalent joint velocity to use as input for the friction model. It also sets the state of the friction compensation.
    * If the active friction model is the OFF, than the friction compensation state is set to NOT_COMPENSATING.
    * If the operator is not requesting a force or an acceleration, or if the current joint velocity is bigger than a threshold the friction compensation state
    * is set to NOT_COMPENSATING. The velocity threshold can be set to avoid compensation when the joint is moving too fast, this can be necessary to avoid over-compensations.
    * If the abs of the filtered current joint velocity is less than the stiction velocity (the joint is in stiction), but the operator is commanding a movement or a force, 
    * the equivalent velocity is computed in two different way based on the value of the requested acceleration.
    * In case the requested acceleration is greater than a threshold, the equivalent velocity is the SMALL_VELOCITY_ABS with the sign of the desired acceleration.
    * If the desired acceleration is less than the threshold then the equivalent velocity is the SMALL_VELOCITY_ABS with the sign of the desired force.
    * If the abs of the filtered current joint velocity is greater than the stiction velocity (the joint is out of stiction) we compute the equivalent velocity in a similar 
    * way but instead of the force we use the current joint velocity. 
    * As current Joint velocity use the less noisy. 
    * This control can be really stiff from if you are trying to apply a force on the joint to move it. This because here are predominant the desired force exerted from the robot to the environment
    * and the desired acceleration.
    * 
    * @return equivalent joint velocity - This velocity is computed to be used in any friction model also when the joint is in stiction.
    * Is an alpha filtered variable to avoid oscillations when the requested force or requested acceleration are close their relative threshold
    * 
    */
   protected Double selectFrictionStateAndFrictionVelocity(double requestedForce, double currentJointVelocity, double requestedJointAcceleration)
   {
      filteredVelocity.update(currentJointVelocity);

      if (activeFrictionModel.getEnumValue() == FrictionModel.OFF)
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if ((Math.abs(requestedJointAcceleration) < accelerationThreshold.getDoubleValue() && Math.abs(requestedForce) < forceThreshold.getDoubleValue())
            || Math.abs(filteredVelocity.getDoubleValue()) > maxJointVelocityToCompensate.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (Math.abs(filteredVelocity.getDoubleValue()) > stictionTransitionVelocity.getDoubleValue())
      {
         if (Math.abs(requestedJointAcceleration) < accelerationThreshold.getDoubleValue())
         {
            frictionCompensationState.set(FrictionState.OUT_ST_VELOCITY);
            velocityForFrictionCalculation.update(currentJointVelocity);
         }
         else
         {
            frictionCompensationState.set(FrictionState.OUT_ST_ACCELERATION_D);
            velocityForFrictionCalculation.update(Math.signum(requestedJointAcceleration) * smallVelocityAbs.getDoubleValue());
         }
      }
      else
      {
         if (Math.abs(requestedJointAcceleration) > accelerationThreshold.getDoubleValue())
         {
            frictionCompensationState.set(FrictionState.IN_ST_ACCELERATION_D);
            velocityForFrictionCalculation.update(Math.signum(requestedJointAcceleration) * smallVelocityAbs.getDoubleValue());
         }
         else
         {
            frictionCompensationState.set(FrictionState.IN_ST_FORCE_D);
            velocityForFrictionCalculation.update(Math.signum(requestedForce) * smallVelocityAbs.getDoubleValue());
         }
      }

      return velocityForFrictionCalculation.getDoubleValue();
   }

   public void setActiveFrictionModel(FrictionModel requestedFrictionModel)
   {
      if ((frictionModels.containsKey(requestedFrictionModel) && frictionModels.get(requestedFrictionModel) != null) && requestedFrictionModel != null)
      {
         if (activeFrictionModel.getEnumValue() != requestedFrictionModel)
         {
            activeFrictionModel.set(requestedFrictionModel);
            checkIfExistFrictionModelForThisJoint(requestedFrictionModel);
         }
      }
      else
      {
         if (requestedFrictionModel != null)
         {
            System.out.println("No model exist for " + requestedFrictionModel.name());
         }
         else
         {
            System.out.println("Requested model is null");
         }
      }
   }  
   
   private double clampValue(double val, double min, double max)
   {
      return Math.max(min, Math.min(max, val));
   }
   
   /**
    * Use this method to adjust the exponential smoothing factor 'alpha' for the equivalent velocity.
    * Increasing this number makes more smooth the transition between different 'FrictionState'.
    * 
    * @param alpha - smoothing factor
    */
   public void setSmoothFactorForEquivalentVelocity(double alpha)
   {
      velocityForFrictionCalculation.setAlpha(alpha);
   }

   /**
    * Use this method to select the effectiveness of the friction compensation.
    * The predicted friction force is increased or reduced by multiplying it with the effectiveness parameter.
    * 
    * @param effectiveness - if 1.0 the effectiveFrictionForce equals the friction force predicted by the active friction model. Currently is clamped between 0.0 and 1.0.
    */
   public void setFrictionCompensationEffectiveness(double effectiveness)
   {
      frictionCompensationEffectiveness.set(clampValue(effectiveness, 0.0, 1.0));
   }

   public FrictionModel getActiveFrictionModel()
   {
      return activeFrictionModel.getEnumValue();
   }

   /**
    * Use this method to get the friction force predicted by the active friction model
    * 
    * @return friction force predicted by the active friction model.
    */
   public double getCurrentFrictionForce()
   {
      return frictionForce.getDoubleValue();
   }

   /**
    * Use this method to get the effective friction force which is obtained by scaling the friction force of the active friction model with the effectiveness parameter.
    * 
    * @return scaled friction force predicted by the active friction model.
    */
   public double getCurrentEffectiveFrictionForce()
   {
      return frictionForce.getDoubleValue() * frictionCompensationEffectiveness.getDoubleValue();
   }

   public FrictionState getCurrentFrictionState()
   {
      return frictionCompensationState.getEnumValue();
   }

   protected JointFrictionModel getActiveJointFrictionModel()
   {
      return frictionModels.get(activeFrictionModel.getEnumValue());
   }
   
   public void setStictionTransitionVelocity(double value)
   {
      stictionTransitionVelocity.set(value);
   }
   
   public void resetVelocityForFrictionCalculation()
   {
      velocityForFrictionCalculation.reset();
   }
   
   public static double getSmallVelocityAbs()
   {
      return SMALL_VELOCITY_ABS;
   }
   
   public static double getAccelerationThreshold()
   {
      return ACCELERATION_THRESHOLD;
   }
   
   public static double getAlphaEquivalentVelocity()
   {
      return ALPHA_EQUIVALENT_VELOCITY;
   }

   protected abstract void checkIfExistFrictionModelForThisJoint(FrictionModel requestedFrictionModel);
}
