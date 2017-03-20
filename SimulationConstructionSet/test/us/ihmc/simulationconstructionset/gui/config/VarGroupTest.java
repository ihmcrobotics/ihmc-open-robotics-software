package us.ihmc.simulationconstructionset.gui.config;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class VarGroupTest {

	String variable1, variable2, variable3, variable4;
	YoVariableRegistry registry;
	VarGroup varGroup;

	@Before
	public void setUp() {
		registry = new YoVariableRegistry("regsitry");
		variable1 = "doubleYoVariable1";
		variable2 = "doubleYoVariable2";
		variable3 = "doubleYoVariable3";
		variable4 = "doubleYoVariable4";
		varGroup = new VarGroup("varGroup");
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testSetAndGetName() {
		assertTrue("varGroup" == varGroup.getName());

		varGroup.setName("varGroupTest1");
		assertTrue("varGroupTest1" == varGroup.getName());

		varGroup.setName("varGroupTestLongNameWithNumbers123456789");
		assertTrue("varGroupTestLongNameWithNumbers123456789" == varGroup
				.getName());
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testAddGetAndRemoveSingleVariable() {
		varGroup.addVar(variable1);

		String[] varGroupArray = varGroup.getVars();
		assertTrue(variable1 == varGroupArray[0]);

		varGroup.removeVar("doubleYoVariable1");
		varGroupArray = varGroup.getVars();
		assertTrue(0 == varGroupArray.length);
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testAddMultipleVariables() {
		String[] variablesToBeAdded = { variable1, variable2, variable3,
				variable4 };

		varGroup.addVars(variablesToBeAdded);
		String[] varGroupArrayReturned = varGroup.getVars();

		for (int i = 0; i < 4; i++) {
			assertTrue(variablesToBeAdded[i] == varGroupArrayReturned[i]);
		}

	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
	public void testAddRemoveAndGetRegularExpressions() {
		String[] regularExpressionsToBeAdded = { variable1, variable2,
				variable3, variable4 };

		varGroup.addRegularExpressions(regularExpressionsToBeAdded);
		String[] regularExpressionsReturned = varGroup.getRegularExpressions();

		for (int i = 0; i < 4; i++) {
			assertTrue(regularExpressionsToBeAdded[i] == regularExpressionsReturned[i]);
		}

		varGroup.removeRegularExpression(variable1);

		regularExpressionsReturned = varGroup.getRegularExpressions();
		String[] regularExpressionsToBeAdded1 = { variable2, variable3,
				variable4 };

		for (int i = 0; i < 3; i++) {
			assertTrue(regularExpressionsToBeAdded1[i] == regularExpressionsReturned[i]);
		}

	}

}
