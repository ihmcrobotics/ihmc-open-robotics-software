package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;

public class SimulationToDispatch
{
   private final Simulation simulation;
   private final String description;
   private String resultsString;

   private final SimulationConstructor constructor;

   private final String[] structuralParameterNames;
   private final double[] structuralParameterValues;
   private final String[] inputStateVariableNames, outputStateVariableNames;
   private double[] inputStateVals;

   private double[] finalState;
   private boolean simulationFinished = false;

   private final DispatchDoneListener dispatchDoneListener;

   public SimulationToDispatch(Simulation simulation, String description, SimulationConstructor constructor, String[] structuralParameterNames,
                               double[] structuralParameterValues, String[] inputStateVars, double[] newVals, String[] outputStateVars,
                               DispatchDoneListener dispatchDoneListener)
   {
      this.simulation = simulation;
      this.description = description;
      this.constructor = constructor;

      this.structuralParameterNames = structuralParameterNames;
      this.structuralParameterValues = structuralParameterValues;

      this.inputStateVariableNames = inputStateVars;
      this.outputStateVariableNames = outputStateVars;

      this.finalState = new double[outputStateVars.length];

      setStateVals(newVals);
      this.dispatchDoneListener = dispatchDoneListener;
   }

   public DispatchDoneListener getDispatchDoneListener()
   {
      return dispatchDoneListener;
   }

   public Simulation getSimulation()
   {
      return simulation;
   }

   public SimulationConstructor getConstructor()
   {
      return constructor;
   }

   public String[] getStructuralParameterNames()
   {
      return structuralParameterNames;
   }

   public double[] getStructuralParameterValues()
   {
      return structuralParameterValues;
   }

   public double[] getFinalState()
   {
      return finalState;
   }

   public void setStateVals(double[] newVals)
   {
      this.inputStateVals = new double[newVals.length];

      for (int i = 0; i < newVals.length; i++)
      {
         inputStateVals[i] = newVals[i];
      }
   }

   public void setResultsString(String resultsString)
   {
      this.resultsString = resultsString;
   }

   public String getResultsString()
   {
      return resultsString;
   }

   public String getDescription()
   {
      return description;
   }

   public void setFinalState(double[] finalState)
   {
      this.finalState = finalState;
   }

   public double[] getInputState()
   {
      return inputStateVals;
   }

   public String[] getInputStateVariableNames()
   {
      return inputStateVariableNames;
   }

   public String[] getOutputStateVariableNames()
   {
      return outputStateVariableNames;
   }

   public void setSimulationFinished()
   {
      this.simulationFinished = true;
   }

   public boolean isSimulationFinished()
   {
      return simulationFinished;
   }

}
