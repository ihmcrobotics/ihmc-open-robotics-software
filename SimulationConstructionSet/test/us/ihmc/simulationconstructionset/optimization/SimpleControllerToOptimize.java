package us.ihmc.simulationconstructionset.optimization;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.utilities.parameterOptimization.DoubleYoVariableParameterToOptimize;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;

public class SimpleControllerToOptimize implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleControllerToOptimize");
   
   private final DoubleYoVariable parameterOne = new DoubleYoVariable("parameterOne", registry);
   private final DoubleYoVariable costFunction = new DoubleYoVariable("costFunction", registry);
   
   private final ListOfParametersToOptimize listOfParametersToOptimize;
   
   public SimpleControllerToOptimize()
   {
      listOfParametersToOptimize = new ListOfParametersToOptimize();
      
      DoubleYoVariableParameterToOptimize parameterOneToOptimize = new DoubleYoVariableParameterToOptimize(-10.0, 10.0, parameterOne, listOfParametersToOptimize);
   }
   
   
   public ListOfParametersToOptimize getListOfParametersToOptimizeForTrialOne()
   {
      return listOfParametersToOptimize;
   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "SimpleControllerToOptimize";
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      costFunction.set((2.0 - parameterOne.getDoubleValue()) * (2.0 - parameterOne.getDoubleValue()));
   }
   
   public double getCost()
   {
      double ret = costFunction.getDoubleValue();

      return ret;
   }

   public void setCurrentValues(ListOfParametersToOptimize listOfParameters)
   {
      this.listOfParametersToOptimize.setCurrentValues(listOfParameters);      
   }


   public void printParameters(ListOfParametersToOptimize listOfParametersToOptimize)
   {
      DoubleYoVariableParameterToOptimize parameter = (DoubleYoVariableParameterToOptimize) listOfParametersToOptimize.get(0);
      System.out.println("parameter = " + parameter.getCurrentValue());
      System.out.println("parameterOne = " + parameterOne.getDoubleValue());
   }


   public boolean verifyParametersCloseToOptimal()
   {
      DoubleYoVariableParameterToOptimize parameter = (DoubleYoVariableParameterToOptimize) listOfParametersToOptimize.get(0);
      if (Math.abs(parameter.getCurrentValue() - 2.0) < 0.01) return true;
      
      return false;
   }

}
