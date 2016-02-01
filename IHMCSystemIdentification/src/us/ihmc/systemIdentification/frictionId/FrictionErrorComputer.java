package us.ihmc.systemIdentification.frictionId;

import java.util.ArrayList;
import java.util.HashMap;

import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.systemIdentification.frictionId.frictionModels.FrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.JointFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.PressureBasedFrictionModel;

/**
 * Use this class to compute the error between measured joint torque and predicted joint torque for a set of samples.
 * The class can also uniformly equalize the number of samples per velocity intervals. Use the dedicated constructor. 
 * If you are not interested in doing identification with the pressure based friction model, the positive and negative pressures can be all zeros.
 * 
 * @param model - instance of JointFrictionModel to identify.
 * @param sampleMap - an HashMap of String and ArrayList<Double> containing all samples. In particular the following quantities are required:
 * @param   "position"
 * @param   "velocity"
 * @param   "acceleration"
 * @param   "negativePressure"
 * @param   "positivePressure"
 * @param   "measuredTorque"
 * 
 * @param uniformSamples - set to true if you want to uniformly equalize the samples.
 * @param numberOfIntervals - specify the number of velocity intervals used to perform sample equalization.
 * @param maxPointPerInterval - specify the maximum number of samples per each interval.
 * 
 */

public class FrictionErrorComputer implements FunctionNtoM
{
   private static final double OFFSET_LOWER_BOUND = -Math.PI;
   private static final double OFFSET_UPPER_BOUND = Math.PI;
   private static final int NUMBER_OF_ADDITIONAL_PARAMETERS = 3;
   private static final String[] EXPECTED_INPUT_VARIABLES = {"position", "velocity", "acceleration", "negativePressure", "positivePressure", "measuredTorque"};
   private static final int NUMBER_OF_REQUIRED_INPUTS = EXPECTED_INPUT_VARIABLES.length;
   
   private JointFrictionModel model;
   private final HashMap<String, ArrayList<Double>> sampleMap;
   private final int numberOfSamples;

   private double frictionTorque, inertiaTorque, gravityTorque;
   private double boundedOffset;
   private double[] frictionParameters;
   
   /**
    * Use this class to compute the error between measured joint torque and predicted joint torque for a set of samples.
    * The class can also uniformly equalize the number of samples per velocity intervals. Use the dedicated constructor. 
    * If you are not interested in doing identification with the pressure based friction model, the positive and negative pressures can be all zeros.
    * 
    * @param model - instance of JointFrictionModel to identify.
    * @param sampleMap - an HashMap of String and ArrayList<Double> containing all samples. In particular the following quantities are required:
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
      this(model, sampleMap, false, 0, 0);
   }

   /**
    * Use this class to compute the error between measured joint torque and predicted joint torque for a set of samples.
    * The class can also uniformly equalize the number of samples per velocity intervals.
    * If you are not interested in doing identification with the pressure based friction model, the positive and negative pressures can be all zeros.
    * 
    * @param model - instance of JointFrictionModel to identify.
    * @param sampleMap - an HashMap of String and ArrayList<Double> containing all samples. In particular the following quantities are required:
    * @param   "position"
    * @param   "velocity"
    * @param   "acceleration"
    * @param   "negativePressure"
    * @param   "positivePressure"
    * @param   "measuredTorque"
    * 
    * @param uniformSamples - set to true if you want to uniformly equalize the samples.
    * @param numberOfIntervals - specify the number of velocity intervals used to perform sample equalization.
    * @param maxPointPerInterval - specify the maximum number of samples per each interval.
    * 
    */
   public FrictionErrorComputer(JointFrictionModel model, HashMap<String, ArrayList<Double>> sampleMap, boolean uniformSamples, int numberOfIntervals, int maxPointPerInterval)
   {
      if (sampleMap.size() != NUMBER_OF_REQUIRED_INPUTS || sampleMap == null)
      {
         throw new IllegalArgumentException("The input sample map is not valid. Check if is null or if number of inputs is not correct.");
      }
      
      if(uniformSamples)
      {
         this.sampleMap = uniformSamplesByVelocity(sampleMap, numberOfIntervals, maxPointPerInterval);
      }
      else
      {
      this.sampleMap = sampleMap;
      }
      
      this.model = model;
      numberOfSamples = this.sampleMap.get("position").size();
   }

   @Override
   public int getNumOfInputsN()
   {
      return model.getNumberOfParameters() + NUMBER_OF_ADDITIONAL_PARAMETERS;
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
      frictionParameters = new double[input.length-NUMBER_OF_ADDITIONAL_PARAMETERS];
      
      for (int j = NUMBER_OF_ADDITIONAL_PARAMETERS; j < input.length; j++)
      {
         frictionParameters[j - NUMBER_OF_ADDITIONAL_PARAMETERS] = input[j];
      }

      model.updateParameters(frictionParameters);

      for (int i = 0; i < numberOfSamples; i++)
      {
         if (model.getFrictionModel() == FrictionModel.PRESSURE_BASED)
         {
            PressureBasedFrictionModel pressureBasedModel = (PressureBasedFrictionModel) model;
            pressureBasedModel.computeFrictionForce(sampleMap.get("velocity").get(i), sampleMap.get("negativePressure").get(i),
                  sampleMap.get("positivePressure").get(i));
         }
         else
         {
            model.computeFrictionForce(sampleMap.get("velocity").get(i));
         }

         frictionTorque = model.getFrictionForce();
         inertiaTorque = sampleMap.get("acceleration").get(i) * input[0];
         boundedOffset = getBoundedOffset(input[2]);
         gravityTorque = input[1] * Math.cos(sampleMap.get("position").get(i) + boundedOffset);
         output[i] = sampleMap.get("measuredTorque").get(i) - frictionTorque - inertiaTorque - gravityTorque;
      }
   }
   
   /**
    * This function can be used to uniform the samples creating velocity intervals between the max and min velocity.
    * For each interval the number of samples is limited to the requested maximum number of samples.
    * If the interval contains more samples than the maximum the algorithm collects randomly only the maximum number of samples.
    * If the interval contains less samples that the maximum the algorithm takes all samples.
    *  
    * @param samples - an HashMap of String and ArrayList<Double> containing all samples. In particular the following quantities are required:
    *                  "position", "velocity", "acceleration", "negativePressure", "positivePressure", "measuredTorque"
    * @param numberOfIntervals - specify the number of intervals between max and min velocity
    * @param maxPointPerInterval - specify the maximum number of samples that will be collected for each velocity interval
    * @return - an HashMap of String and ArrayList<Double> containing the new samples.
    */
   private HashMap<String, ArrayList<Double>> uniformSamplesByVelocity(HashMap<String, ArrayList<Double>> samples, int numberOfIntervals, int maxPointPerInterval)
   {
      HashMap<String, ArrayList<Double>> ret = new HashMap<String, ArrayList<Double>>();
      for (int i = 0; i < NUMBER_OF_REQUIRED_INPUTS; i++)
      {
         ret.put(EXPECTED_INPUT_VARIABLES[i], new ArrayList<Double>());
      }

      double maxVelocity = getMaxValue(samples.get("velocity"));
      double minVelocity = getMinValue(samples.get("velocity"));
      double delta = (maxVelocity - minVelocity)/numberOfIntervals;
      
      for(int i = 0; i < numberOfIntervals; i++)
      {
         double lowerBound = minVelocity + i * delta;
         double upperBound = minVelocity + (i+1) * delta;
         ArrayList<Integer> indexList = findIndexOfValuesBetweenBounds(samples.get("velocity"), upperBound, lowerBound);
         
         if(indexList.size() > maxPointPerInterval)
         {
            indexList = pickElementsRandomly(indexList, maxPointPerInterval);
         }
         
         for(int j = 0; j < NUMBER_OF_REQUIRED_INPUTS; j++)
         {
            for(int k = 0; k < indexList.size(); k++)
            {
               ret.get(EXPECTED_INPUT_VARIABLES[j]).add(samples.get(EXPECTED_INPUT_VARIABLES[j]).get(indexList.get(k)));               
            }   
         }
      }
      
      return ret;
   }
   
   private ArrayList<Integer> pickElementsRandomly(ArrayList<Integer> list, int maxNumberOfElements)
   {
      ArrayList<Integer> newList = new ArrayList<Integer>();
      boolean enoughElementsSelected = false;
      
      while(!enoughElementsSelected)
      {
         int index = (int) Math.round(Math.random() * (list.size()-1));
         
         if(!newList.contains(list.get(index)))
         {
            newList.add(list.get(index));
            list.remove(index);
         }
         
         if(newList.size() >= maxNumberOfElements)
         {
            enoughElementsSelected = true;
         }
      }
      
      return newList;
   }
   
   private ArrayList<Integer> findIndexOfValuesBetweenBounds(ArrayList<Double> list, double max, double min)
   {
      ArrayList<Integer> indexList = new ArrayList<Integer>();
      
      for(int i = 0; i < list.size(); i++)
      {
         if(list.get(i) < max && list.get(i) > min)
         {
            indexList.add(i);
         }
      }
      
      return indexList;
   }
   
   private double getMaxValue(ArrayList<Double> list)
   {
      double max = Double.NEGATIVE_INFINITY;
      
      for (int i = 0; i < list.size(); i++)
      {
         if (list.get(i) > max) 
         {
           max = list.get(i);
         }
      }
      
      return max;
   }
   
   private double getMinValue(ArrayList<Double> list)
   {
      double min = Double.POSITIVE_INFINITY;
      
      for (int i = 0; i < list.size(); i++)
      {
         if (list.get(i) < min) 
         {
           min = list.get(i);
         }
      }
      
      return min;
   }

   static public synchronized double getBoundedOffset(double value)
   {
      return OFFSET_LOWER_BOUND + (OFFSET_UPPER_BOUND - OFFSET_LOWER_BOUND) * (Math.sin(value) + 1) / 2;
   }
   
   static public int getNumberOfAdditionalParameters()
   {
      return NUMBER_OF_ADDITIONAL_PARAMETERS;
   }
}
