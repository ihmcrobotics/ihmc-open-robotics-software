package us.ihmc.exampleSimulations.springflamingo;

import static org.junit.Assert.fail;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class SpringFlamingoSimulationTest
{
   private SimulationGUITestFixture testFixture;
   private SimulationConstructionSet scs;

   @Before
   public void setUp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      SpringFlamingoSimulation springFlamingoSimulation = new SpringFlamingoSimulation();
      scs = springFlamingoSimulation.getSimulationConstructionSet();

//      ThreadTools.sleep(1000L);
      testFixture = new SimulationGUITestFixture(scs);
   }

   @After
   public void tearDown()
   {
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 8.5)
	@Test(timeout = 42000)
   public void testSpringFlamingoSimulationAndGUI() throws SimulationExceededMaximumTimeException
   {
      testFixture.showWindow();
//      testFixture.clickFileMenu();
//
//      testFixture.selectNameSpaceTab();
//      ThreadTools.sleep(1000);

//      testFixture.selectNewGraphWindowMenu();

      testFixture.selectSearchTab();

      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");

      testFixture.selectVariableAndSetValueInSearchTab("q_pitch", 1.0);
//      ThreadTools.sleep(500);
      testFixture.selectVariableInSearchTab("q_x");
//      ThreadTools.sleep(500);

      testFixture.clickRemoveEmptyGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();

      testFixture.selectVariableInSearchTab("q_z");
      testFixture.middleClickInNthGraph(1);
      testFixture.removeVariableFromNthGraph("q_z", 1);

//      testFixture.selectNewGraphWindowMenu();

//      testFixture.sel
//      testFixture.selectVariableInSearchTab("q_z");
//      ThreadTools.sleep(3000);


//      testFixture.clickRemoveEmptyGraphButton();
//      testFixture.clickNewGraphButton();


//

//      testFixture.clickNewGraphButton();
//      testFixture.clickSimulateButton();
//
//      ThreadTools.sleep(500);
//      testFixture.clickStopButton();
//      testFixture.clickPlayMenu(); //Button();
//
//      ThreadTools.sleep(500);
//      testFixture.clickStopButton();
//
//      testFixture.clickGotoInPointButton();
//
//      ThreadTools.sleep(500);
//
//      int index = scs.getIndex();
//      int inPoint = scs.getInPoint();
//      assertEquals(index, inPoint);
//
//      testFixture.clickPlaybackPropertiesMenu();

//
//    ThreadTools.sleep(1000);
//    ThreadTools.sleepForever();
//    window.menuItem("foo").

   }


	@ContinuousIntegrationTest(estimatedDuration = 3.6)
	@Test(timeout = 30000)
	public void testRewindability() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
	{
      int numTicksToTest = 1000;
	   int numTicksToSimulateAhead = 100;

	   SimulationConstructionSet scs1 = setupScs();
	   SimulationConstructionSet scs2 = setupScs();

	   ArrayList<String> exceptions = new ArrayList<String>();
	   exceptions.add("gc_");
	   exceptions.add("toolFrame");
	   exceptions.add("ef_");
	   exceptions.add("kp_");
	   exceptions.add("TimeNano");
	   exceptions.add("DurationMilli");
	   exceptions.add("startTime");
	   exceptions.add("timePassed");
	   exceptions.add("actualEstimatorDT");
	   exceptions.add("actualControlDT");
	   exceptions.add("TimerCurrent");
	   exceptions.add("TimerAverage");
	   exceptions.add("TimerMovingAverage");
	   exceptions.add("TimerMaximum");
	   exceptions.add("nextExecutionTime");
	   exceptions.add("lastControllerClockTime");
	   exceptions.add("totalDelay");
      exceptions.add("TimerStandardDeviation");

	   SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);

	   double maxDifferenceAllowed = 1e-7;
	   ArrayList<VariableDifference> variableDifferences;getClass();
	   variableDifferences = checker.checkRewindabilityWithSimpleMethod(numTicksToSimulateAhead, numTicksToTest, maxDifferenceAllowed);
	   if (!variableDifferences.isEmpty())
	   {
	      System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
	      fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
	   }

	   scs1.closeAndDispose();
	   scs2.closeAndDispose();
	}

   private SimulationConstructionSet setupScs() throws SimulationExceededMaximumTimeException
   {
      SpringFlamingoRobot springFlamingoConstructor = new SpringFlamingoRobot("SpringFlamingo");
      Robot springFlamingo = springFlamingoConstructor.getRobot();

      double gravity = 9.81;
      SpringFlamingoFastWalkingController controller = new SpringFlamingoFastWalkingController(springFlamingoConstructor, gravity , "springFlamingoFastWalkingController");
      springFlamingo.setController(controller);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(false);
      parameters.setShowWindows(false);
      parameters.setShowSplashScreen(false);

      SimulationConstructionSet scs = new SimulationConstructionSet(springFlamingo, parameters);
      scs.setDT(0.0001, 10);

      return scs;
   }


}
