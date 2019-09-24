package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.linearDynamicSystems.LinearDynamicSystem;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * YoLinearDynamicSystem. Class for simulating a Linear Dynamic System and recording the state,
 * input, and output variables as YoVariables.
 * 
 * @author JerryPratt
 */
public class YoLinearDynamicSystem
{
   private final LinearDynamicSystem linearDynamicSystem;

   private final YoDouble[] stateVariables;
   private final YoDouble[] inputVariables;
   private final YoDouble[] outputVariables;

   public YoLinearDynamicSystem(LinearDynamicSystem linearDynamicSystem, String statePrefix, String inputPrefix, String outputPrefix,
                                YoVariableRegistry registry)
   {
      this.linearDynamicSystem = linearDynamicSystem;
      int order = linearDynamicSystem.getOrder();
      int inputSize = linearDynamicSystem.getInputSize();
      int outputSize = linearDynamicSystem.getOutputSize();

      stateVariables = new YoDouble[order];
      inputVariables = new YoDouble[inputSize];
      outputVariables = new YoDouble[outputSize];

      for (int i = 0; i < order; i++)
      {
         stateVariables[i] = new YoDouble(statePrefix + i, registry);
      }

      for (int i = 0; i < inputSize; i++)
      {
         inputVariables[i] = new YoDouble(inputPrefix + i, registry);
      }

      for (int i = 0; i < outputSize; i++)
      {
         outputVariables[i] = new YoDouble(outputPrefix + i, registry);
      }
   }

   public YoDouble[] getStateVariables()
   {
      return stateVariables;
   }

   public YoDouble[] getInputVariables()
   {
      return inputVariables;
   }

   public YoDouble[] getOuptutVariables()
   {
      return outputVariables;
   }

   public void update(double[] input, double dt)
   {
      double[] currentState = getState();

      setVariables(inputVariables, input);
      double[] nextState = linearDynamicSystem.eulerIntegrateOneStep(currentState, input, dt);
      setState(nextState);

      double[] output = linearDynamicSystem.getOutputFromState(nextState, input);
      setVariables(outputVariables, output);
   }

   public void setState(double[] state)
   {
      setVariables(stateVariables, state);
   }

   private static void setVariables(YoDouble[] variables, double[] values)
   {
      if (variables.length != values.length)
      {
         throw new RuntimeException("variables.length != values.length");
      }

      for (int i = 0; i < variables.length; i++)
      {
         variables[i].set(values[i]);
      }
   }

   private static double[] getVariables(YoDouble[] variables)
   {
      double[] values = new double[variables.length];

      for (int i = 0; i < variables.length; i++)
      {
         values[i] = variables[i].getValue();
      }
      return values;
   }

   public double[] getState()
   {
      return getVariables(stateVariables);
   }

   public double[] getOutput()
   {
      return getVariables(outputVariables);
   }

   public double[] getInput()
   {
      return getVariables(inputVariables);
   }

}
