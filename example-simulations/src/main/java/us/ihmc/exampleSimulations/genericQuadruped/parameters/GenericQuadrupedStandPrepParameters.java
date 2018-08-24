package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedOrderedJointMap;

import java.util.HashMap;

public class GenericQuadrupedStandPrepParameters implements WholeBodySetpointParameters
{
   private static HashMap<String, Double> setPoints = new HashMap<>();

   static
   {
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_RIGHT_HIP_ROLL.getName(), 0.6);
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_RIGHT_HIP_PITCH.getName(), 0.8);
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_RIGHT_KNEE_PITCH.getName(), -1.4);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_RIGHT_HIP_ROLL.getName(), 0.6);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_RIGHT_HIP_PITCH.getName(), 0.8);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_RIGHT_KNEE_PITCH.getName(), -1.4);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_LEFT_HIP_ROLL.getName(), 0.6);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_LEFT_HIP_PITCH.getName(), 0.8);
      setSetpoint(GenericQuadrupedOrderedJointMap.HIND_LEFT_KNEE_PITCH.getName(), -1.4);
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_LEFT_HIP_ROLL.getName(), 0.6);
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_LEFT_HIP_PITCH.getName(), 0.8);
      setSetpoint(GenericQuadrupedOrderedJointMap.FRONT_LEFT_KNEE_PITCH.getName(), -1.4);
   }

   private static void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(int jointIndex)
   {
      return getSetpoint(GenericQuadrupedOrderedJointMap.values[jointIndex].getName());
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if(setPoints.containsKey(jointName))
      {
         return setPoints.get(jointName);
      }
      else
      {
         throw new RuntimeException("Stand prep setpoint not set for: " + jointName);
      }
   }
}
