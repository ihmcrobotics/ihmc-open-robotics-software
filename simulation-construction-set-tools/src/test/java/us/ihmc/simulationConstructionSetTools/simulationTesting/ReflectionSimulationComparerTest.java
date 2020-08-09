package us.ihmc.simulationConstructionSetTools.simulationTesting;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.lang.reflect.Field;
import java.util.Collection;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReflectionSimulationComparerTest
{

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @AfterEach
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@Test
   public void testTwoEmptySimulations()
   {
      ReflectionSimulationComparer comparer = new ReflectionSimulationComparer(Integer.MAX_VALUE, Integer.MAX_VALUE);

      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(100);
      SimulationConstructionSet scs0 = new SimulationConstructionSet(new Robot("Null"), parameters);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(new Robot("Null"), parameters);
      
      boolean simulationsAreTheSame = comparer.compare(scs0, scs1);
      assertFalse(simulationsAreTheSame);
      simulationsAreTheSame = comparer.compare(scs0, scs1);
      assertFalse(simulationsAreTheSame);

      Collection<Field> differingFields = comparer.getDifferingFields();

      assertEquals(2, differingFields.size());
      for (Field field : differingFields)
      {
        String fieldName = field.getName();
        assertTrue(fieldName.equals("hash"));
      }
      
      comparer = new ReflectionSimulationComparer(Integer.MAX_VALUE, Integer.MAX_VALUE);
      
      comparer.addFieldsToIgnore(differingFields);
      
      simulationsAreTheSame = comparer.compare(scs0, scs1);
     
      assertTrue(simulationsAreTheSame);
   }

	@Test
   public void testTwoRewindableSimulationsWithAScript() throws IllegalArgumentException, SecurityException, IllegalAccessException, NoSuchFieldException, UnreasonableAccelerationException
   {      
      Robot robot0 = createSimpleRobot();
      RobotController rewindableController0 = new RewindableOrNotRewindableController(true);
      robot0.setController(rewindableController0);
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(100);
      SimulationConstructionSet scs0 = new SimulationConstructionSet(robot0, parameters);
      scs0.setDT(0.0001, 11);
      scs0.startOnAThread();

      Robot robot1 = createSimpleRobot();
      RobotController rewindableController1 = new RewindableOrNotRewindableController(true);
      robot1.setController(rewindableController1);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, parameters);
      scs1.setDT(0.0001, 11);
      scs1.startOnAThread();

      int nTicksInitial = 121; 
      int nTicksCompare = 121; 
      int nTicksFinal = 11; 
      SimulationComparisonScript script = new SimpleRewindabilityComparisonScript(nTicksInitial, nTicksCompare, nTicksFinal);
      ReflectionSimulationComparer.compareTwoSimulations(scs0, scs1, script, true, true);
   }


   private Robot createSimpleRobot()
   {
      Robot robot0 = new Robot("robot");
      FloatingJoint floatingJoint0 = new FloatingJoint("floatingJoint", new Vector3D(), robot0);
      Link link0 = new Link("body");
      link0.setMass(1.0);
      link0.setMomentOfInertia(0.1, 0.1, 0.1);
      floatingJoint0.setLink(link0);
      robot0.addRootJoint(floatingJoint0);
      return robot0;
   }

	@Test
   public void testTwoNonRewindableSimulationsWithAScript() throws IllegalArgumentException, SecurityException, IllegalAccessException, NoSuchFieldException, UnreasonableAccelerationException
   {      
      Robot robot0 = new Robot("robot");
      RobotController rewindableController0 = new RewindableOrNotRewindableController(false);
      robot0.setController(rewindableController0);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(100);
      SimulationConstructionSet scs0 = new SimulationConstructionSet(robot0, parameters);
      scs0.setDT(0.0001, 10);
      scs0.startOnAThread();

      Robot robot1 = new Robot("robot");
      RobotController rewindableController1 = new RewindableOrNotRewindableController(false);
      robot1.setController(rewindableController1);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, parameters);
      scs1.setDT(0.0001, 10);
      scs1.startOnAThread();

      int nTicksInitial = 20; 
      int nTicksCompare = 30; 
      int nTicksFinal = 40; 
      SimulationComparisonScript script = new SimpleRewindabilityComparisonScript(nTicksInitial, nTicksCompare, nTicksFinal);
      ReflectionSimulationComparer.compareTwoSimulations(scs0, scs1, script, false, true);
   }
   
   private class RewindableOrNotRewindableController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry("RewindableObject");
      
      private final YoInteger counter = new YoInteger("counter", registry);
      private int counter2;
     
      private final boolean isRewindable;
      
      public RewindableOrNotRewindableController(boolean isRewindable)
      {
         this.isRewindable = isRewindable;
      }

      @Override
      public void initialize()
      {         
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "RewindableController";
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public void doControl()
      {
         counter.increment();
         if (!isRewindable) counter2++;
      }
   }
   
   

}
