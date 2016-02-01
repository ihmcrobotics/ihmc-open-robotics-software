package us.ihmc.simulationconstructionset.optimization;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;

public class SimpleSimulationIndividualToEvaluate extends IndividualToEvaluate
{
   private static final boolean DEBUG = false;

   private final ListOfParametersToOptimize listOfParametersToOptimize;
   
   private final Simulation simulation;
   private final SimpleControllerToOptimize controller;
   
   private boolean isEvaluationDone = false;
   private double cost;
   
   public SimpleSimulationIndividualToEvaluate()
   {      
      SimpleRobotToOptimize robot = new SimpleRobotToOptimize();
      controller = new SimpleControllerToOptimize();
      robot.setController(controller);
      
      simulation = new Simulation(robot, 1);
      
      listOfParametersToOptimize = controller.getListOfParametersToOptimizeForTrialOne(); 
   }

   public IndividualToEvaluate createNewIndividual()
   {
      return new SimpleSimulationIndividualToEvaluate();
   }

   public void startEvaluation()
   {
      controller.setCurrentValues(listOfParametersToOptimize);
      
      try
      {
         simulation.simulate(1.0);
      } 
      catch (UnreasonableAccelerationException e)
      {
         System.out.println("SimpleSimulationToOptimize Crashed. Setting cost to Infinity");

         cost = Double.POSITIVE_INFINITY;
         isEvaluationDone = true;
         return;
      }

      cost = controller.getCost();
      if (DEBUG)
         System.out.println("SimpleSimulationToOptimize Cost: " + cost);
      isEvaluationDone = true;
   }

   public boolean isEvaluationDone()
   {
      return isEvaluationDone;
   }

   public double computeFitness()
   {
      return cost;
   }

   public ListOfParametersToOptimize getStructuralParametersToOptimize()
   {
      return null;
   }
   
   public ListOfParametersToOptimize getControlParametersToOptimize()
   {
      return listOfParametersToOptimize;
   }

}
