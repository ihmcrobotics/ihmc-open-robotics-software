package us.ihmc.darpaRoboticsChallenge.frictionCompensation;

import java.util.EnumMap;

import us.ihmc.utilities.frictionModels.FrictionModel;
import us.ihmc.utilities.frictionModels.FrictionState;
import us.ihmc.utilities.frictionModels.JointFrictionModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

public abstract class JointFrictionModelsHolder
{
   private final DoubleYoVariable stictionTransitionVelocity;
   private final AlphaFilteredYoVariable filteredVelocity;
   private final DoubleYoVariable alphaForFilteredVelocity;
   private final DoubleYoVariable forceThreshold;
   private final DoubleYoVariable frictionCompensationEffectiveness;
   private final DoubleYoVariable maxJointVelocityToCompensate;

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
      this.forceThreshold.set(forceThreshold);
      filteredVelocity = new AlphaFilteredYoVariable(name + "_alphaFilteredVelocity", registry, alphaForFilteredVelocity);
      filteredVelocity.update(0.0);
      frictionCompensationEffectiveness = new DoubleYoVariable(name + "_frictionCompensationEffectiveness", registry);
      frictionCompensationEffectiveness.set(0.0);
      this.maxJointVelocityToCompensate = new DoubleYoVariable(name + "_maxJointVelocityToCompensate", registry);
      this.maxJointVelocityToCompensate.set(maxJointVelocityToCompensate);
   }

   /**
    * This method computes an equivalent joint velocity to use as input for the friction model. It also sets the state of the friction compensation.
    * If the active friction model is the OFF, than the friction compensation state is set to NOT_COMPENSATING.
    * If the operator is not requesting a force or a velocity, or if the current joint velocity is bigger than a threshold the friction compensation state
    * is set to NOT_COMPENSATING. The velocity threshold can be set to avoid compensation when the joint is moving too fast, this can be necessary to avoid over-compensations.
    * If the abs of the filtered current joint velocity is bigger than the stiction velocity than the velocity that will be used to compute the friction force is the current joint velocity.
    * Instead if the abs of the filtered current joint velocity is less than the stiction velocity (the joint is in stiction, but the operator is commanding a movement or a force), 
    * the equivalent velocity is computed in two different way based on the value of the requested force.
    * In case the requested force is greater than a force threshold, the equivalent velocity is computed as the stiction velocity with the sign of the desired force.
    * In case of velocity the equivalent velocity is the desired velocity. 
    * As current Joint velocity use the less noisy. 
    * In case of stiction the discrimination between force or velocity mode is done based on the requested force value, so force control is predominant.
    * 
    */
   protected Double selectFrictionStateAndFrictionVelocity(double requestedForce, double currentJointVelocity, double requestedJointVelocity)
   {
      double velocityForFrictionCalculation;
      filteredVelocity.update(currentJointVelocity);

      if (activeFrictionModel.getEnumValue() == FrictionModel.OFF)
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if ((requestedJointVelocity == 0.0 && Math.abs(requestedForce) < forceThreshold.getDoubleValue())
            || Math.abs(filteredVelocity.getDoubleValue()) > maxJointVelocityToCompensate.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (Math.abs(filteredVelocity.getDoubleValue()) > stictionTransitionVelocity.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.OUT_STICTION);
         velocityForFrictionCalculation = currentJointVelocity;
      }
      else
      {
         if (Math.abs(requestedForce) < forceThreshold.getDoubleValue())
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_VELOCITY_MODE);
            velocityForFrictionCalculation = requestedJointVelocity;
         }
         else
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_FORCE_MODE);
            velocityForFrictionCalculation = stictionTransitionVelocity.getDoubleValue() * Math.signum(requestedForce);
         }
      }

      return velocityForFrictionCalculation;
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

   /**
    * Use this method to select the effectiveness of the friction compensation.
    * The predicted friction force is increased or reduced by multiplying it with the effectiveness parameter.
    * 
    * @param effectiveness - if 1.0 the effectiveFrictionForce equals the friction force predicted by the active friction model.
    */
   public void setFrictionCompensationEffectiveness(double effectiveness)
   {
      frictionCompensationEffectiveness.set(effectiveness);
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

   protected abstract void checkIfExistFrictionModelForThisJoint(FrictionModel requestedFrictionModel);
}
