package us.ihmc.darpaRoboticsChallenge.frictionCompensation;

import java.util.EnumMap;

import us.ihmc.utilities.frictionModels.FrictionModel;
import us.ihmc.utilities.frictionModels.FrictionState;
import us.ihmc.utilities.frictionModels.JointFrictionModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;

public abstract class JointFrictionModelsHolder
{
   private final String name;
   private final DoubleYoVariable stictionTransitionVelocity;

   protected final DoubleYoVariable frictionForce;
   protected final EnumYoVariable<FrictionState> frictionCompensationState;
   protected final EnumYoVariable<FrictionModel> activeFrictionModel;
   protected final EnumMap<FrictionModel, JointFrictionModel> frictionModels;

   public JointFrictionModelsHolder(String name, YoVariableRegistry registry)
   {
      this.name = name;
      frictionModels = new EnumMap<FrictionModel, JointFrictionModel>(FrictionModel.class);
      frictionCompensationState = new EnumYoVariable<FrictionState>(name + "_frictionCompensationState", registry, FrictionState.class);
      activeFrictionModel = new EnumYoVariable<FrictionModel>(name + "_activeFrictionModel", registry, FrictionModel.class);
      frictionForce = new DoubleYoVariable(name + "_frictionForce", registry);
      stictionTransitionVelocity = new DoubleYoVariable(name + "_stictionTransitionVelocity", registry);
   }

   /**
    * This method computes an equivalent joint velocity in case the joint is in stiction, but the operator is commanding a movement or a force.
    * In case of force, the equivalent velocity is computed as the stiction velocity with the sign of the desired force.
    * In case of velocity the equivalent velocity is the desired velocity. 
    * As current Joint velocity use the less noisy.
    */
   protected Double selectFrictionStateAndFrictionVelocity(double requestedForce, double currentJointVelocity, double requestedJointVelocity)
   {
      double velocityForFrictionCalculation;

      if (activeFrictionModel.getEnumValue() == FrictionModel.OFF)
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (requestedJointVelocity == 0.0 && requestedForce == 0.0)
      {
         frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
         frictionForce.set(0.0);
         return null;
      }

      if (Math.abs(currentJointVelocity) > stictionTransitionVelocity.getDoubleValue())
      {
         frictionCompensationState.set(FrictionState.OUT_STICTION);
         velocityForFrictionCalculation = currentJointVelocity;
      }
      else
      {
         if (requestedJointVelocity == 0.0)
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_FORCE_MODE);
            velocityForFrictionCalculation = stictionTransitionVelocity.getDoubleValue() * Math.signum(requestedForce);
         }
         else
         {
            frictionCompensationState.set(FrictionState.IN_STICTION_VELOCITY_MODE);
            velocityForFrictionCalculation = requestedJointVelocity;
         }
      }

      return velocityForFrictionCalculation;
   }

   public void setActiveFrictionModel(FrictionModel requestedFrictionModel)
   {
      activeFrictionModel.set(requestedFrictionModel);
      checkIfExistFrictionModelForThisJoint(requestedFrictionModel);
      stictionTransitionVelocity.set(getActiveJointFrictionModel().getStictionTransitionVelocity());
   }

   public FrictionModel getActiveFrictionModel()
   {
      return activeFrictionModel.getEnumValue();
   }

   public double getCurrentFrictionForce()
   {
      return frictionForce.getDoubleValue();
   }
   
   public FrictionState getCurrentFrictionState()
   {
      return frictionCompensationState.getEnumValue();
   }

   protected JointFrictionModel getActiveJointFrictionModel()
   {
      return frictionModels.get(activeFrictionModel.getEnumValue());
   }
   
   protected abstract void checkIfExistFrictionModelForThisJoint(FrictionModel requestedFrictionModel);
}
