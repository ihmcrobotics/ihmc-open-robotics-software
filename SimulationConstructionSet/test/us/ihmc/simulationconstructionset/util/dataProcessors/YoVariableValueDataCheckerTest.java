package us.ihmc.simulationconstructionset.util.dataProcessors;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
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
         ThreadTools.sleepForever();
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

      YoVariableValueDataChecker maximumFirstDerivativeDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime());

      maximumFirstDerivativeDataChecker.setMaximumDerivative(1.1);
      maximumFirstDerivativeDataChecker.setMaximumSecondDerivate(1.1);
      maximumFirstDerivativeDataChecker.setMaximumValue(1.1);
      maximumFirstDerivativeDataChecker.setMinimumValue(-1.1);

      scs.applyDataProcessingFunction(maximumFirstDerivativeDataChecker);
      assertTrue(!maximumFirstDerivativeDataChecker.isMaxDerivativeExeeded());
      assertTrue(!maximumFirstDerivativeDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(!maximumFirstDerivativeDataChecker.isMaxValueExeeded());
      assertTrue(!maximumFirstDerivativeDataChecker.isMinValueExeeded());
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

      YoVariableValueDataChecker maximumFirstDerivativeDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime());
      maximumFirstDerivativeDataChecker.setMaximumDerivative(0.9);
      maximumFirstDerivativeDataChecker.setMaximumSecondDerivate(0.9);
      maximumFirstDerivativeDataChecker.setMaximumValue(0.9);
      maximumFirstDerivativeDataChecker.setMinimumValue(-0.9);

      scs.applyDataProcessingFunction(maximumFirstDerivativeDataChecker);

      assertTrue(maximumFirstDerivativeDataChecker.isMaxDerivativeExeeded());
      assertTrue(maximumFirstDerivativeDataChecker.isMaxSecondDerivativeExeeded());
      assertTrue(maximumFirstDerivativeDataChecker.isMaxValueExeeded());
      assertTrue(maximumFirstDerivativeDataChecker.isMinValueExeeded());
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


      YoVariableValueDataChecker maximumFirstDerivativeDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime());

      maximumFirstDerivativeDataChecker.setMaximumValue(1.0);
      maximumFirstDerivativeDataChecker.setMinimumValue(2.0);
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


      YoVariableValueDataChecker maximumFirstDerivativeDataChecker = new YoVariableValueDataChecker(scs, position, robot.getYoTime());

      maximumFirstDerivativeDataChecker.setMinimumValue(2.0);
      maximumFirstDerivativeDataChecker.setMaximumValue(1.0);
   }
}
