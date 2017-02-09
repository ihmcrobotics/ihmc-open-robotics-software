package us.ihmc.simulationconstructionset.util.simulationTesting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.MemoryTools;

public class AllYoVariablesSimulationComparerTest 
{
	Robot robot1;
	Robot robot2;
	
	YoVariableRegistry rootRegistry1;
	YoVariableRegistry rootRegistry2;
	
	DoubleYoVariable doubleYoVariable1;
	DoubleYoVariable doubleYoVariable2;
	DoubleYoVariable doubleYoVariable3;
	DoubleYoVariable doubleYoVariable4;
	
	DoubleYoVariable doubleYoVariableA12;

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
	
	@Before
	public void setUp()
	{
		robot1 = createSimpleRobotOne("robot"); 
		robot2 = createSimpleRobotTwo("robot");

		rootRegistry1 = new YoVariableRegistry("rootRegistry");
		rootRegistry2 = new YoVariableRegistry("rootRegistry");

		doubleYoVariable1 = new DoubleYoVariable("doubleYoVariableA1", rootRegistry1);
		doubleYoVariable2 = new DoubleYoVariable("doubleYoVariableA2", rootRegistry1);
		
		// Only put in registry 1;
		doubleYoVariableA12 = new DoubleYoVariable("doubleYoVariableA12", rootRegistry1);
		
		doubleYoVariable3 = new DoubleYoVariable("doubleYoVariableA1", rootRegistry2);
		doubleYoVariable4 = new DoubleYoVariable("doubleYoVariableA2", rootRegistry2);
		
		// Only put in registry 2;
		new DoubleYoVariable("aIgnore", rootRegistry2);
		new DoubleYoVariable("zIgnore", rootRegistry2);
	}
	
	private Robot createSimpleRobotOne(String name)
	{
		Robot robot = new Robot(name);

		PinJoint joint1 = new PinJoint("joint", new Vector3d(0.0, 0.0, 0.0), robot, Axis.Y);
		Link link1 = link1();
		joint1.setLink(link1);
		robot.addRootJoint(joint1);
		
		joint1.setInitialState(0.11, 0.22);
		joint1.setTau(33.3);
		return robot;
	}
	
	private Robot createSimpleRobotTwo(String name)
	{
		Robot robot = new Robot(name);

		PinJoint joint2 = new PinJoint("joint", new Vector3d(0.0, 0.0, 0.0), robot, Axis.Y);
		Link link2 = link2();
		joint2.setLink(link2);
		robot.addRootJoint(joint2);
		
		joint2.setInitialState(0.11, 0.22);
		joint2.setTau(33.3);
		return robot;
	}
	
	private Link link1()
	   {
	      Link ret = new Link("link1");
	      ret.setMass(1.0);
	      ret.setComOffset(2.0, 0.0, 0.0);
	      ret.setMomentOfInertia(0.0, 3.0, 0.0);

	      return ret;
	   }
	
	private Link link2()
	   {
	      Link ret = new Link("link2");
	      ret.setMass(2.0);
	      ret.setComOffset(2.0, 0.0, 0.0);
	      ret.setMomentOfInertia(0.0, 5.0, 0.0);

	      return ret;
	   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
	public void testCompareWithZeroEpsilon() 
	{
		AllYoVariablesSimulationComparer comparerWithZeroEpsilon = new AllYoVariablesSimulationComparer(0.0);
		
		SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
		scsParameters.setCreateGUI(false);
		
		SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, scsParameters);
		SimulationConstructionSet scs2 = new SimulationConstructionSet(robot2, scsParameters);
		scs1.getRootRegistry().addChild(rootRegistry1);
		scs2.getRootRegistry().addChild(rootRegistry2);
		
		doubleYoVariable1.set(1.0);
		doubleYoVariable2.set(2.0);
		doubleYoVariable3.set(1.0);
		doubleYoVariable4.set(2.0);
		
		assertFalse(comparerWithZeroEpsilon.compare(scs1, scs2)); 
		comparerWithZeroEpsilon.addException("doubleYoVariableA12");
		comparerWithZeroEpsilon.addException("Ignore");
		assertTrue(comparerWithZeroEpsilon.compare(scs1, scs2)); 

		doubleYoVariable1.set(1.0);
		doubleYoVariable2.set(3.0);
		doubleYoVariable3.set(1.0);
		doubleYoVariable4.set(2.0);
		assertFalse(comparerWithZeroEpsilon.compare(scs1, scs2));
		ArrayList<YoVariable<?>[]> differences = comparerWithZeroEpsilon.getDifferences();
		assertEquals(1, differences.size());
		assertEquals(doubleYoVariable2, differences.get(0)[0]);
		assertEquals(doubleYoVariable4, differences.get(0)[1]);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
	public void testWithLowEpsilon()
	{
		AllYoVariablesSimulationComparer comparerWithLowEpsilon = new AllYoVariablesSimulationComparer(0.01);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(false);
		SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, parameters);
		SimulationConstructionSet scs2 = new SimulationConstructionSet(robot2, parameters);
		scs1.getRootRegistry().addChild(rootRegistry1);
		scs2.getRootRegistry().addChild(rootRegistry2);
		
		doubleYoVariable1.set(1.8);
		doubleYoVariable2.set(2.0);
		doubleYoVariable3.set(1.791);
		doubleYoVariable4.set(2.009);
		assertFalse(comparerWithLowEpsilon.compare(scs1, scs2));
		comparerWithLowEpsilon.addException("doubleYoVariableA12");
		comparerWithLowEpsilon.addException("Ignore");
		assertTrue(comparerWithLowEpsilon.compare(scs1, scs2));

		doubleYoVariable1.set(1.80);
		doubleYoVariable2.set(2.0);
		doubleYoVariable3.set(1.79);
		doubleYoVariable4.set(2.009);
		assertFalse(comparerWithLowEpsilon.compare(scs1, scs2)); 
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout = 30000)
	public void testWithHighEpsilon()
	{
	   AllYoVariablesSimulationComparer comparerWithLargeEpsilon = new AllYoVariablesSimulationComparer(5.0);

	   SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
	   parameters.setCreateGUI(false);

	   SimulationConstructionSet scs1 = new SimulationConstructionSet(robot1, parameters);
	   SimulationConstructionSet scs2 = new SimulationConstructionSet(robot2, parameters);
	   scs1.getRootRegistry().addChild(rootRegistry1);
		scs2.getRootRegistry().addChild(rootRegistry2);
		
		doubleYoVariable1.set(98.56);
		doubleYoVariable2.set(97.01);
		doubleYoVariable3.set(94.98);
		doubleYoVariable4.set(92.02);
		assertFalse(comparerWithLargeEpsilon.compare(scs1, scs2));
		comparerWithLargeEpsilon.addException("doubleYoVariableA12");
		comparerWithLargeEpsilon.addException("Ignore");
		assertTrue(comparerWithLargeEpsilon.compare(scs1, scs2));

		doubleYoVariable1.set(97.02);
		doubleYoVariable2.set(95.01);
		doubleYoVariable3.set(9);
		doubleYoVariable4.set(92.02);
		assertFalse(comparerWithLargeEpsilon.compare(scs1, scs2));
	}
}
