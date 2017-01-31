package us.ihmc.simulationconstructionset.util.simulationRunner;


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class SimulationRewindabilityVerifierTest
{
   private static final boolean SHOW_GUI = false;
   private static final boolean VERBOSE = false;
   private final double DT = 0.01;

	@ContinuousIntegrationTest(estimatedDuration = 4.2)
	@Test(timeout=300000)
   public void testRewindableSimulation() throws UnreasonableAccelerationException
   {
      SimulationConstructionSet scs1 = constructRewindableSimulationConstructionSet();
      SimulationConstructionSet scs2 = constructRewindableSimulationConstructionSet();
      
      ArrayList<String> exceptions = new ArrayList<String>();
//      exceptions.add("ticks_till_control");
      SimulationRewindabilityVerifier verifier = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      
      int numTests = 5000;
      double maxDifferenceAllowed = 1e-12;
      
      ArrayList<VariableDifference> variableDifferences = verifier.checkRewindabilityWithSimpleMethod(numTests, maxDifferenceAllowed);
      assertTrue(variableDifferences.isEmpty());
      
      scs1.closeAndDispose();
      scs2.closeAndDispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 4.2)
	@Test(timeout=300000)
   public void testEasilyDetectableNonRewindableSimulation() throws UnreasonableAccelerationException
   {
      SimulationConstructionSet scs1 = constructEasilyDetectableNonRewindableSimulationConstructionSet();
      SimulationConstructionSet scs2 = constructEasilyDetectableNonRewindableSimulationConstructionSet();
      
      ArrayList<String> exceptions = new ArrayList<String>();
//      exceptions.add("ticks_till_control");
      SimulationRewindabilityVerifier verifier = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      
      int numTests = 5000;
      double maxDifferenceAllowed = 1e-12;
      
      ArrayList<VariableDifference> variableDifferences = verifier.checkRewindabilityWithSimpleMethod(numTests, maxDifferenceAllowed);
      
      if (VERBOSE)
      {
         System.out.println("\ntestEasilyDetectableNonRewindableSimulation differences:");
         System.out.println(VariableDifference.allVariableDifferencesToString(variableDifferences));
      }
      assertEquals(2, variableDifferences.size());
      
      scs1.closeAndDispose();
      scs2.closeAndDispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 8.5)
	@Test(timeout=300000)
   public void testDifficultToDetectNonRewindableSimulation() throws UnreasonableAccelerationException
   {
      SimulationConstructionSet scs1 = constructDifficultToDetectNonRewindableSimulationConstructionSet();
      SimulationConstructionSet scs2 = constructDifficultToDetectNonRewindableSimulationConstructionSet();
      
      ArrayList<String> exceptions = new ArrayList<String>();
//      exceptions.add("ticks_till_control");
      SimulationRewindabilityVerifier verifier = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      
      int numTests = 4000;
      int numTicksAhead = 800;
      
      double maxDifferenceAllowed = 1e-12;
      
      ArrayList<VariableDifference> variableDifferences = verifier.checkRewindabilityWithSimpleMethod(numTests, maxDifferenceAllowed);
      
      if (VERBOSE)
      {
         System.out.println("\ntestDifficultToDetectNonRewindableSimulation differences with simple method:");
         System.out.println(VariableDifference.allVariableDifferencesToString(variableDifferences));
      }

      assertTrue(variableDifferences.isEmpty()); // Make sure the test is hard enough that the simple method doesn't catch it.

      // Then reconstruct and try again:
      scs1 = constructDifficultToDetectNonRewindableSimulationConstructionSet();
      scs2 = constructDifficultToDetectNonRewindableSimulationConstructionSet();
      verifier = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);

      int numTicksToStartComparingAt = 1;
      variableDifferences = verifier.checkRewindabilityWithRigorousMethod(numTicksToStartComparingAt, numTests, numTicksAhead, maxDifferenceAllowed);

      if (VERBOSE)
      {
         System.out.println("\ntestDifficultToDetectNonRewindableSimulation differences with complex method:");
         System.out.println(VariableDifference.allVariableDifferencesToString(variableDifferences));
      }

      assertEquals(2, variableDifferences.size());
      
      scs1.closeAndDispose();
      scs2.closeAndDispose();
   }
   
   private SimulationConstructionSet constructRewindableSimulationConstructionSet()
   {
      Robot robot = new Robot("Test");
      RobotController controller = new RewindableController(robot);
      robot.setController(controller);
      
      return constructSimulationConstructionSet(robot, controller);
   }
   
   
   private SimulationConstructionSet constructEasilyDetectableNonRewindableSimulationConstructionSet()
   {
      Robot robot = new Robot("Test");
      RobotController controller = new EasilyDetectableNonRewindableController(robot);
      robot.setController(controller);
      
      return constructSimulationConstructionSet(robot, controller);
   }
   
   private SimulationConstructionSet constructDifficultToDetectNonRewindableSimulationConstructionSet()
   {
      Robot robot = new Robot("Test");
      RobotController controller = new DifficultToDetectNonRewindableController(robot);
      robot.setController(controller);
      
      return constructSimulationConstructionSet(robot, controller);
   }
   
   private SimulationConstructionSet constructSimulationConstructionSet(Robot robot, RobotController controller)
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters); 
      scs.setDT(DT, 1);
      
      Thread thread = new Thread(scs);
      thread.start();

      try
      {
         Thread.sleep(2000);
      } 
      catch (InterruptedException e)
      {
      }

      return scs;
   }

   private static class RewindableController implements RobotController
   {
      protected final YoVariableRegistry registry = new YoVariableRegistry("controller");
      
      protected final DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", registry);
      protected final DoubleYoVariable variableTwo = new DoubleYoVariable("variableTwo", registry);
      protected final DoubleYoVariable variableThree = new DoubleYoVariable("variableThree", registry);
      protected final DoubleYoVariable variableFour = new DoubleYoVariable("variableFour", registry);

      protected final Robot robot;
      
      public RewindableController(Robot robot)
      {
         this.robot = robot;
      }
      
      @Override
      public void doControl()
      {      
         variableOne.set(Math.cos(robot.getTime()));
         variableTwo.set(robot.getTime());
         variableThree.set(variableOne.getDoubleValue());
         variableFour.set(1.2);
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
        return "Test";
      }
      
      @Override
      public void initialize()
      {      
      }

      @Override
      public String getDescription()
      {
         return getName();
      }
      
   }
   
   
   private static class EasilyDetectableNonRewindableController extends RewindableController
   {
      private double nonRegisteredVariable;
      
      public EasilyDetectableNonRewindableController(Robot robot)
      {
         super(robot);
      }
      
      @Override
      public void doControl()
      {
         super.doControl();
         
         if (robot.getTime() >= 1.0)
         {
            nonRegisteredVariable = nonRegisteredVariable + 1.0;
            variableTwo.set(nonRegisteredVariable);
            variableFour.set(nonRegisteredVariable * 2.0);
         }
      }
   }
   
   private static class DifficultToDetectNonRewindableController extends RewindableController
   {
      private double nonRegisteredVariable = 5;
     
      private double lastTimeChanged;
      private double timeBetweenChanges = 3.0;
      
      private DoubleYoVariable lastTimeUpdated = new DoubleYoVariable("lastTimeUpdated", registry);
      private double timeBetweenUpdates = 5.0;
      
      public DifficultToDetectNonRewindableController(Robot robot)
      {
         super(robot);
      }
      
      @Override
      public void doControl()
      {
         super.doControl();
         
         if (robot.getTime() > lastTimeChanged + timeBetweenChanges)
         {
            lastTimeChanged = robot.getTime();
            nonRegisteredVariable = nonRegisteredVariable + 1.0;
            
//            System.out.println("Changed nonRegisteredVariable to " + nonRegisteredVariable + ", time = " + robot.getTime());
         }
         
         if (robot.getTime() > lastTimeUpdated.getDoubleValue() + timeBetweenUpdates)
         {
            lastTimeUpdated.set(robot.getTime());
            
            variableTwo.set(nonRegisteredVariable);
            variableThree.set(3.0);
            variableFour.set(nonRegisteredVariable * 2);
            
//            System.out.println("Changed a bunch of variables. time = " + robot.getTime());
         }
      }
   }
   
}
