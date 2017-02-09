package us.ihmc.simulationconstructionset.simulationDispatcher.client;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;

public abstract class DispatchedSimulationIndividualToEvaluate extends IndividualToEvaluate
{
   private boolean evalDone = false;
   private double fitness;

   private final String[] outputStateVariableNames;
   private final SimulationDispatcher simulationDispatcher;
   private final Simulation simulation;
   private final SimulationConstructor simulationConstructor;

   public DispatchedSimulationIndividualToEvaluate(String[] outputStateVariableNames, SimulationDispatcher simulationDispatcher, Simulation simulation,
         SimulationConstructor simulationConstructor)
   {
      this.outputStateVariableNames = outputStateVariableNames;
      this.simulationDispatcher = simulationDispatcher;
      this.simulation = simulation;
      this.simulationConstructor = simulationConstructor;
   }

   @Override
   public void startEvaluation()
   {
      //    System.out.println("Starting Evaluation of SliderIndividual" + this.getName());

      ListOfParametersToOptimize controlParametersToOptimize = this.getControlParametersToOptimize();
      ListOfParametersToOptimize structuralParametersToOptimize = this.getStructuralParametersToOptimize();

      double[] controlParameterValues = controlParametersToOptimize.getValuesAsDoubles();
      String[] controlParameterNames = controlParametersToOptimize.getNames();

      double[] structuralParameterValues = structuralParametersToOptimize.getValuesAsDoubles();
      String[] structuralParameterNames = structuralParametersToOptimize.getNames();

      DispatchDoneListener listener = new DispatchDoneListener()
      {
         @Override
         public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState)
         {
            fitness = computeFitnessAfterDispatch(finalState);

            System.out.println("Simulation Done.  Fitness = " + fitness);

            dispatchSim.setResultsString("fitness: " + fitness);
            evalDone = true;
         }
      };

      SimulationToDispatch dispatchSliderSim = new SimulationToDispatch(simulation, this.getName(), simulationConstructor, structuralParameterNames,
            structuralParameterValues, controlParameterNames, controlParameterValues, outputStateVariableNames, listener);

      simulationDispatcher.addSimulation(dispatchSliderSim);

   }

   @Override
   public boolean isEvaluationDone()
   {
      return evalDone;
   }

   @Override
   public double computeFitness()
   {
      return fitness;
   }
   
   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append(getStructuralParametersToOptimize().toString());
      builder.append(getControlParametersToOptimize().toString());
      builder.append("fitness: " + this.getFitness());
      
      return builder.toString();
   }

   public abstract double computeFitnessAfterDispatch(double[] finalState);

}
