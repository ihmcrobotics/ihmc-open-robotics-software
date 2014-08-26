package us.ihmc.darpaRoboticsChallenge.frictionCompensation;

import java.util.EnumMap;

import us.ihmc.utilities.frictionModels.FrictionModel;
import us.ihmc.utilities.frictionModels.JointFrictionModel;
import us.ihmc.utilities.frictionModels.JointFrictionModel.FrictionState;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class JointFrictionModelsHolder
{
   private final String name;
   private final EnumYoVariable<FrictionState> frictionCompensationState;
   private final DoubleYoVariable frictionForce;
   private EnumYoVariable<FrictionModel> activeFrictionModel;
   protected final EnumMap<FrictionModel, JointFrictionModel> frictionModels;
   
   private double stictionTransitionVelocity;
   
   public JointFrictionModelsHolder(String name, YoVariableRegistry registry)
   {
      this.name = name;
      frictionModels = new EnumMap<FrictionModel, JointFrictionModel>(FrictionModel.class);
      frictionCompensationState = new EnumYoVariable<FrictionState>(name + "_frictionCompensationState", registry, FrictionState.class);
      activeFrictionModel = new EnumYoVariable<FrictionModel>(name + "_activeFrictionModel", registry, FrictionModel.class);
      frictionForce = new DoubleYoVariable(name + "_frictionForce", registry);
   }

   
   /**
    * This method adds the friction force to improve the force control 
    * As current Joint velocity use the preTransmission because is less noisy
    */
    public double compensateFrictionForForceControl(double requestedForce, double currentJointVelocity, double requestedJointVelocity, double negPressure,
          double posPressure)
    {
       if(activeFrictionModel.getEnumValue() == FrictionModel.OFF)
       {
          frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
          frictionForce.set(0.0);
          return requestedForce;        
       }
       
       if (requestedJointVelocity == 0.0 && requestedForce == 0.0)
       {
          frictionCompensationState.set(FrictionState.NOT_COMPENSATING);
          frictionForce.set(0.0);
          return 0.0;
       }

       if (Math.abs(currentJointVelocity) > stictionTransitionVelocity)
       {
          frictionCompensationState.set(FrictionState.OUT_STICTION);
          frictionForce.set(getActiveJointFrictionModel().getFrictionForce());
       }
       else
       {
          if (requestedJointVelocity == 0.0)
          {
             frictionCompensationState.set(FrictionState.IN_STICTION_FORCE_MODE);
             frictionForce.set(getActiveJointFrictionModel().getFrictionForce());
          }
          else
          {
             frictionCompensationState.set(FrictionState.IN_STICTION_VELOCITY_MODE);
             frictionForce.set(getActiveJointFrictionModel().getFrictionForce());
          }
       }

       return requestedForce + frictionForce.getDoubleValue();
    }

    
    public void setActiveFrictionModel(FrictionModel requestedFrictionModel)
    {
//       jointFrictionParameter = AtlasJointFrictionParameters.getJoitFrictionParameter(requestedFrictionModel, jointId);
//       checkIfJointHasRequestedFrictionModelParameters(requestedFrictionModel, jointId);
    }
    
   public FrictionModel getActiveFrictionModel()
   {
      return activeFrictionModel.getEnumValue();
   }
   
   protected JointFrictionModel getActiveJointFrictionModel()
   {
      return frictionModels.get(activeFrictionModel.getEnumValue());
   }
}
