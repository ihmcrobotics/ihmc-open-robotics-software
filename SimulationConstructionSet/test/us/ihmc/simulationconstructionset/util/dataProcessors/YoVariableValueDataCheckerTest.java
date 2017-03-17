package us.ihmc.simulationconstructionset.util.dataProcessors;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class YoVariableValueDataCheckerTest
{
   private double EPSILON = 1e-10;

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         //ThreadTools.sleepForever();
      }

      simulationTestingParameters = null;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

      double amplitude = 3.0;
      double offset = 7.0;
      double freq = 5.0;

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(amplitude * Math.sin(freq * time) + offset);

         scs.tickAndUpdate();
      }

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(amplitude * freq * 1.01);
      valueDataCheckerParameters.setMaximumSecondDerivative(amplitude * freq * freq * 1.01);
      valueDataCheckerParameters.setMaximumValue(amplitude + offset + 1.0);
      valueDataCheckerParameters.setMinimumValue(offset - amplitude - 1.0);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);
      yoVariableValueDataChecker.cropFirstPoint();

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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


      double amplitude = 3.0;
      double offset = 7.0;
      double freq = 5.0;

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(amplitude * Math.sin(freq * time) + offset);
         velocity.set(amplitude* freq* Math.cos(freq * time));

         scs.tickAndUpdate();
      }


      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(amplitude * freq * 1.01);
      valueDataCheckerParameters.setMaximumSecondDerivative(amplitude * freq * freq * 1.01);
      valueDataCheckerParameters.setMaximumValue(amplitude + offset + 1.0);
      valueDataCheckerParameters.setMinimumValue(offset - amplitude - 1.0);
      valueDataCheckerParameters.setErrorThresholdOnDerivativeComparison(1e-1);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters, velocity);
      yoVariableValueDataChecker.cropFirstPoint();

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isDerivativeCompErrorOccurred());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

      double amplitude = 3.0;
      double offset = 7.0;
      double freq = 5.0;

      Random random = new Random(1776L);

      for (double time = 0.0; time < 7.0; time = time + deltaT)
      {
         robot.setTime(time);

         position.set(amplitude * Math.sin(freq * time) + offset);
         velocity.set(amplitude* freq* Math.cos(freq * time)+ 0.01 * (random.nextDouble() - 0.5));

         scs.tickAndUpdate();
      }

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();
      valueDataCheckerParameters.setMaximumDerivative(amplitude * freq * 1.01);
      valueDataCheckerParameters.setMaximumSecondDerivative(amplitude * freq * freq * 1.01);
      valueDataCheckerParameters.setMaximumValue(amplitude + offset + 1.0);
      valueDataCheckerParameters.setMinimumValue(offset - amplitude - 1.0);
      valueDataCheckerParameters.setErrorThresholdOnDerivativeComparison(1e-2);

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters, velocity);
      yoVariableValueDataChecker.cropFirstPoint();

      scs.applyDataProcessingFunction(yoVariableValueDataChecker);
      assertTrue(!yoVariableValueDataChecker.isMaxDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!yoVariableValueDataChecker.isMaxValueExeeded());
      assertTrue(!yoVariableValueDataChecker.isMinValueExeeded());
      assertTrue(yoVariableValueDataChecker.isDerivativeCompErrorOccurred());
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSimpleSmoothDerviativeExceed()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();
      simulationConstructionSetParameters.setCreateGUI(false);
      simulationConstructionSetParameters.setShowSplashScreen(false);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationConstructionSetParameters);

//      scs.hideViewport();
      scs.startOnAThread();

      double deltaT = 0.001;

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


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
      public void testErrorThresholdOnDerivativeComparison()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      Random random = new Random(1776L);
      double value = random.nextDouble();
      yoVariableValueDataChecker.setErrorThresholdOnDerivativeComparison(value);
      assertEquals(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getErrorThresholdOnDerivativeComparison(), value, EPSILON);

      yoVariableValueDataChecker.setErrorThresholdOnDerivativeComparison(-value);
      assertEquals(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getErrorThresholdOnDerivativeComparison(), value, EPSILON);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
      public void testMaximumDerivative()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      Random random = new Random(1776L);
      double value = random.nextDouble();
      yoVariableValueDataChecker.setMaximumDerivative(value);
      assertEquals(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getMaximumDerivative(), value, EPSILON);

      yoVariableValueDataChecker.setMaximumDerivative(-value);
      assertEquals(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getMaximumDerivative(), value, EPSILON);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaximumSecondDerivative()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      Random random = new Random(1776L);
      double value = random.nextDouble();
      valueDataCheckerParametersOriginal.setMaximumSecondDerivative(value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumSecondDerivative(), value, EPSILON);

      valueDataCheckerParametersOriginal.setMaximumSecondDerivative(-value);
      assertEquals(valueDataCheckerParametersOriginal.getMaximumSecondDerivative(), value, EPSILON);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
      public void testMaximumValue()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      Random random = new Random(1776L);
      double value = random.nextDouble();
      yoVariableValueDataChecker.setMaximumValue(value);
      assertEquals(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getMaximumValue(), value, EPSILON);

      yoVariableValueDataChecker.setMaximumValue(-value);
      assertFalse(yoVariableValueDataChecker.getValueDataCheckerParametersCopy().getMaximumValue() == value);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMinimumValue()
   {
      ValueDataCheckerParameters valueDataCheckerParametersOriginal = new ValueDataCheckerParameters();

      Random random = new Random(1776L);

      double value = random.nextDouble();
      valueDataCheckerParametersOriginal.setMinimumValue(value);
      assertEquals(valueDataCheckerParametersOriginal.getMinimumValue(), value, EPSILON);

      valueDataCheckerParametersOriginal.setMinimumValue(-value);
      assertFalse(valueDataCheckerParametersOriginal.getMinimumValue() == value);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testSetMinGreaterThanMax()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      double value = 10.0;
      yoVariableValueDataChecker.setMaximumValue(value);
      yoVariableValueDataChecker.setMinimumValue(value + 1.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testSetMaxLessThanMin()
   {
      Robot robot = new Robot("Derivative");

      YoVariableRegistry registry = new YoVariableRegistry("variables");
      DoubleYoVariable position = new DoubleYoVariable("position", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      ValueDataCheckerParameters valueDataCheckerParameters = new ValueDataCheckerParameters();

      YoVariableValueDataChecker yoVariableValueDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime(), valueDataCheckerParameters);

      double value = 10.0;
      yoVariableValueDataChecker.setMinimumValue(value);
      yoVariableValueDataChecker.setMaximumValue(value - 10.0);
   }

}
