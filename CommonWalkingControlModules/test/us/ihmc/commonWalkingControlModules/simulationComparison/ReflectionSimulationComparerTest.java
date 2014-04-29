package us.ihmc.commonWalkingControlModules.simulationComparison;

import com.yobotics.simulationconstructionset.*;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.simulationTesting.ReflectionSimulationComparer;
import com.yobotics.simulationconstructionset.util.simulationTesting.SimpleRewindabilityComparisonScript;
import com.yobotics.simulationconstructionset.util.simulationTesting.SimulationComparisonScript;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.utilities.MemoryTools;

import javax.vecmath.Vector3d;
import java.lang.reflect.Field;
import java.util.Collection;

import static org.junit.Assert.*;

public class ReflectionSimulationComparerTest
{

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @Test
   public void testTwoEmptySimulations()
   {
      ReflectionSimulationComparer comparer = new ReflectionSimulationComparer(Integer.MAX_VALUE, Integer.MAX_VALUE);
      
      SimulationConstructionSet scs0 = new SimulationConstructionSet(new Robot("Null"), false, 100);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(new Robot("Null"), false, 100);
      
      boolean simulationsAreTheSame = comparer.compare(scs0, scs1);
      assertFalse(simulationsAreTheSame);
      simulationsAreTheSame = comparer.compare(scs0, scs1);
      assertFalse(simulationsAreTheSame);

      Collection<Field> differingFields = comparer.getDifferingFields();
      
      assertEquals(4, differingFields.size());
      for (Field field : differingFields)
      {
        String fieldName = field.getName();
        assertTrue((fieldName.equals("hash")) || (fieldName.equals("nnuId")));
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
      SimulationConstructionSet scs0 = new SimulationConstructionSet(robot0, false, 100);
      scs0.setDT(0.0001, 11);
      scs0.startOnAThread();

      Robot robot1 = createSimpleRobot();
      RobotController rewindableController1 = new RewindableOrNotRewindableController(true);
      robot1.setController(rewindableController1);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, false, 100);
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
      FloatingJoint floatingJoint0 = new FloatingJoint("floatingJoint", new Vector3d(), robot0);
      Link link0 = new Link("body");
      link0.setMass(1.0);
      link0.physics.setMomentOfInertia(0.1, 0.1, 0.1);
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
      SimulationConstructionSet scs0 = new SimulationConstructionSet(robot0, false, 100);
      scs0.setDT(0.0001, 10);
      scs0.startOnAThread();

      Robot robot1 = new Robot("robot");
      RobotController rewindableController1 = new RewindableOrNotRewindableController(false);
      robot1.setController(rewindableController1);
      SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, false, 100);
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
      private final YoVariableRegistry registry = new YoVariableRegistry("RewindableObject");
      
      private final IntegerYoVariable counter = new IntegerYoVariable("counter", registry);
      private int counter2;
     
      private final boolean isRewindable;
      
      public RewindableOrNotRewindableController(boolean isRewindable)
      {
         this.isRewindable = isRewindable;
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
         return "RewindableController";
      }

      public String getDescription()
      {
         return getName();
      }

      public void doControl()
      {
         counter.increment();
         if (!isRewindable) counter2++;
      }
   }
   
   

}
