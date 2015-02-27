package us.ihmc.simulationconstructionset.util.dataProcessors;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Fast})
public class YoVariableValueDataCheckerTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();


   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         //ThreadTools.sleepForever();
      }
   }


   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000)
   public void testSimpleSmoothDerviativeNoExeeded()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

      scs.hideViewport();
      scs.startOnAThread();

      double deltaT = 0.001;

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(Math.sin(time));

         scs.tickAndUpdate();
      }
      

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(1.1);
      valueDataCheckerParameters.setMaximumSecondDerivative(1.1);
      valueDataCheckerParameters.setMaximumValue(1.1);
      valueDataCheckerParameters.setMinimumValue(-1.1);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
   }
   
   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000)
   public void testSimpleSmoothDerviativeNoExeededWithSecondDerivateProvided()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);
      DoubleYoVariable velocity = new DoubleYoVariable("velocity", registry);

      
      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

      scs.hideViewport();
      scs.startOnAThread();

      double deltaT = 0.001;

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(Math.sin(time));
         velocity.set(Math.cos(time));

         scs.tickAndUpdate();
      }
      
      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(1.1);
      valueDataCheckerParameters.setMaximumSecondDerivative(1.1);
      valueDataCheckerParameters.setMaximumValue(1.1);
      valueDataCheckerParameters.setMinimumValue(-1.1);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters, velocity);

     

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isDerivativeCompErrorOccurred());
   }
   
   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000)
   public void testSimpleSmoothDerviativeNoExeededWithSecondDerivateProvidedAndError()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);
      DoubleYoVariable velocity = new DoubleYoVariable("velocity", registry);

      
      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

      scs.hideViewport();
      scs.startOnAThread();

      double deltaT = 0.001;

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(Math.sin(time));
         velocity.set(Math.cos(time) + 0.01 * (Math.random() - 0.5));

         scs.tickAndUpdate();
      }

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(1.1);
      valueDataCheckerParameters.setMaximumSecondDerivative(1.1);
      valueDataCheckerParameters.setMaximumValue(1.1);
      valueDataCheckerParameters.setMinimumValue(-1.1);
      valueDataCheckerParameters.setErrorThresholdOnDerivativeComparison(1e-2);
      
      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters, velocity);

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isDerivativeCompErrorOccurred());
   }


   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000)
   public void testSimpleSmoothDerviativeExceed()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.hideViewport();
      scs.startOnAThread();

      double deltaT = 0.001;

      scs.startOnAThread();

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(Math.sin(time));

         scs.tickAndUpdate();
      }
      
      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(0.9);
      valueDataCheckerParameters.setMaximumSecondDerivative(0.9);
      valueDataCheckerParameters.setMaximumValue(0.9);
      valueDataCheckerParameters.setMinimumValue(-0.9);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);

      assertTrue(yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(yoVariableValueDataChecker.isMinValueExeeded());
   }


   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000, expected=RuntimeException.class)
   public void testMinGreaterThanMax()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

      scs.hideViewport();
      scs.startOnAThread();

      
      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);
      
      yoVariableValueDataChecker.setMaximumValue(1.0);
      yoVariableValueDataChecker.setMinimumValue(2.0);
   }
   
   @AverageDuration(duration = 2.0)
   @Test(timeout = 300000, expected=RuntimeException.class)
   public void testMaxGreaterThanMin() 
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

      scs.hideViewport();
      scs.startOnAThread();

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      yoVariableValueDataChecker.setMinimumValue(2.0);
      yoVariableValueDataChecker.setMaximumValue(1.0);
   }
}
