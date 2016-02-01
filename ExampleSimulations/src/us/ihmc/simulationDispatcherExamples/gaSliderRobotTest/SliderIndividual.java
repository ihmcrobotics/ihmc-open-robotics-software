package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchedSimulationIndividualToEvaluate;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.utilities.parameterOptimization.DoubleParameterToOptimize;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;

public class SliderIndividual extends DispatchedSimulationIndividualToEvaluate
{
   private final ListOfParametersToOptimize controlParametersToOptimize, structuralParametersToOptimize;
   
   private final DoubleParameterToOptimize k1, k2, k3, k4, q_joint1, q_joint2;
   
   private final SimulationDispatcher simulationDispatcher;
   private final Simulation simulation;
   private final SimulationConstructor simulationConstructor;


   private static final String[] sliderOutputStateVars = new String[]
   {
      "t", "q_joint1", "q_joint2", "qd_joint1", "qd_joint2", "k1", "k2", "k3", "k4"
   };
   
   public SliderIndividual(SimulationDispatcher simulationDispatcher, Simulation simulation, SimulationConstructor simulationConstructor)
   {
      super(sliderOutputStateVars, simulationDispatcher, simulation, simulationConstructor);
      
      this.controlParametersToOptimize = new ListOfParametersToOptimize();
      
      int  bitsOfResolution = 8;
      k1 = new DoubleParameterToOptimize("k1", -200.0, 200.0, bitsOfResolution, controlParametersToOptimize); 
      k2 = new DoubleParameterToOptimize("k2", -200.0, 200.0, bitsOfResolution, controlParametersToOptimize); 
      k3 = new DoubleParameterToOptimize("k3", -200.0, 200.0, bitsOfResolution, controlParametersToOptimize); 
      k4 = new DoubleParameterToOptimize("k4", -200.0, 200.0, bitsOfResolution, controlParametersToOptimize); 
      
      q_joint1 = new DoubleParameterToOptimize("q_joint1", -0.2, -0.2, 1, controlParametersToOptimize); 
      q_joint2 = new DoubleParameterToOptimize("q_joint2", -0.3, -0.3, 1, controlParametersToOptimize); 

      this.structuralParametersToOptimize = new ListOfParametersToOptimize();

      new DoubleParameterToOptimize("structuralParameterOne", -1.0, 1.0, structuralParametersToOptimize);
      new DoubleParameterToOptimize("structuralParameterTwo", 10.0, 20.0, structuralParametersToOptimize);
      
      this.simulationDispatcher = simulationDispatcher;
      this.simulation = simulation;
      this.simulationConstructor = simulationConstructor;
   }

   public IndividualToEvaluate createNewIndividual()
   {
      return new SliderIndividual(this.simulationDispatcher, this.simulation, this.simulationConstructor);
   }

   public ListOfParametersToOptimize getStructuralParametersToOptimize()
   {
      return structuralParametersToOptimize;
   }

   public ListOfParametersToOptimize getControlParametersToOptimize()
   {
      return controlParametersToOptimize;
   }

   public double computeFitnessAfterDispatch(double[] finalState)
   {
      double fitness = finalState[0];
      double q_joint1 = finalState[1];
      double q_joint2 = finalState[2];
      double qd_joint1 = finalState[3];
      double qd_joint2 = finalState[4];

      if (fitness > 9.9)
      {
         if (Math.abs(q_joint1) < 0.1)
            fitness = fitness + 1.0;
         if (Math.abs(q_joint2) < 0.2)
            fitness = fitness + 1.0;
      }

      return fitness;
   }

}
