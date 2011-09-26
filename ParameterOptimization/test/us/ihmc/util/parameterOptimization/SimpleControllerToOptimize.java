package us.ihmc.util.parameterOptimization;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class SimpleControllerToOptimize implements RobotController
{
   private static final long serialVersionUID = -4500294743640952142L;

   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleControllerToOptimize");
   
   private final DoubleYoVariable parameterOne = new DoubleYoVariable("parameterOne", registry);
   private final DoubleYoVariable costFunction = new DoubleYoVariable("costFunction", registry);
   
   public ListOfParametersToOptimize getListOfParametersToOptimizeForTrialOne()
   {
      ListOfParametersToOptimize ret = new ListOfParametersToOptimize();
      
      DoubleYoVariableParameterToOptimize parameterOneToOptimize = new DoubleYoVariableParameterToOptimize(-10.0, 10.0, parameterOne);
      ret.addParameterToOptimize(parameterOneToOptimize);
      
      return ret;
   }
   
   public void initialize()
   {      
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "SimpleControllerToOptimize";
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      System.out.println("SimpleControllerToOptimize: doControl");
      costFunction.set((2.0 - parameterOne.getDoubleValue()) * (2.0 - parameterOne.getDoubleValue()));
      System.out.println("SimpleControllerToOptimize: doControl. costFunction = " + costFunction.getDoubleValue());

   }
   
   public double getCost()
   {
      double ret = costFunction.getDoubleValue();
      System.out.println("SimpleControllerToOptimize: getCost(): " + ret);

      return ret;
   }

}
