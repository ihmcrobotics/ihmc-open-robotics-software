package us.ihmc.FrictionID;

import java.util.ArrayList;
import java.util.HashMap;

import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.utilities.frictionModels.FrictionModel;
import us.ihmc.utilities.frictionModels.JointFrictionModel;
import us.ihmc.utilities.frictionModels.PressureBasedFrictionModel;
import us.ihmc.utilities.frictionModels.PressureBasedFrictionModel;

public class FrictionErrorComputer implements FunctionNtoM
{
   private static final double OFFSET_LOWER_BOUND = -Math.PI;
   private static final double OFFSET_UPPER_BOUND = -Math.PI;
   
   private JointFrictionModel model;
   private final HashMap<String, ArrayList<Double>> sampleMap;
   private final int numberOfSamples;
   
   private double frictionTorque, inertiaTorque, gravityTorque;
   private double boundedOffset;
   private double[] frictionParameters;
   
   /**
    * Use this class to compute the error between measured joint torque and predicted joint torque for a set of samples.
    * 
    * @param model - instance of JointFrictionModel to identify
    * @param sampleMap - an HashMap of String and ArrayList<Double> containing all samples. In particular the following quantities are required:
    * @param   "time"
    * @param   "position"
    * @param   "velocity"
    * @param   "acceleration"
    * @param   "negativePressure"
    * @param   "positivePressure"
    * @param   "measuredTorque"
    * 
    */

   public FrictionErrorComputer(JointFrictionModel model, HashMap<String, ArrayList<Double>> sampleMap)
   {
      this.model = model;
      this.sampleMap = sampleMap;

      numberOfSamples = sampleMap.get("time").size();
   }

   @Override
   public int getNumOfInputsN()
   {
      return model.getNumberOfParameters() + 1;
   }

   @Override
   public int getNumOfOutputsM()
   {
      return numberOfSamples;
   }

   /**
    * This method computes the error between the measured joint torques and the predicted from the friction model.
    * Are also included an inertia parameter and a gravitational component.
    * The angle offset for the gravitational component is already bounded between -pi and +pi. So to obtain the real 
    * value use the method getBoundedOffset(argument) using as argument the optimization result.
    * 
    * @param inputs are - Inertia + mass*lever arm + angle offset + friction model parameters 
    * @param input[0] - is Inertia
    * @param input[1] - is mass*lever arm
    * @param input[2] - is angle offset
    * @param input[3-end] - model based friction parameters
    */
   @Override
   public void process(double[] input, double[] output)
   {
      for (int j = 3; j < input.length; j++)
      {
         frictionParameters[j-3] = input[j];
      }
      
      model.updateParameters(frictionParameters);
      
      for(int i = 0; i <numberOfSamples; i++)
      {
         if (model.getFrictionModel() == FrictionModel.PRESSURE_BASED)
         {
            PressureBasedFrictionModel pressureBasedModel = (PressureBasedFrictionModel) model;
            pressureBasedModel.computeFrictionForce(sampleMap.get("velocity").get(i),
                                                    sampleMap.get("negativePressure").get(i), 
                                                    sampleMap.get("positivePressure").get(i));
         }
         else
         {
            model.computeFrictionForce(sampleMap.get("velocity").get(i));
         }

         frictionTorque = model.getFrictionForce();
         inertiaTorque = sampleMap.get("acceleration").get(i) * input[0];
         boundedOffset = OFFSET_LOWER_BOUND + (OFFSET_UPPER_BOUND - OFFSET_LOWER_BOUND) * (Math.sin(input[2]) + 1)/2;
         gravityTorque = input[1] * Math.cos(sampleMap.get("position").get(i) + boundedOffset);
         output[i] =  sampleMap.get("measuredTorque").get(i) - frictionTorque - inertiaTorque - gravityTorque;
      }
   }
   
   public double getBoundedOffset(double value)
   {
      return OFFSET_LOWER_BOUND + (OFFSET_UPPER_BOUND - OFFSET_LOWER_BOUND) * (Math.sin(value) + 1)/2;
   }
}
