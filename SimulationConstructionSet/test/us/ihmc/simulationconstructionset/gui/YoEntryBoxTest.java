package us.ihmc.simulationconstructionset.gui;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class YoEntryBoxTest
{
   public enum BadGreekEnum {ALPHA, BETA, GAMMA}

   public enum LargeEnum {THIS_IS_LARGE, THIS_IS_LARGER, A_LARGER_STILL_ENUMERATION_STRING,
           A_YET_LARGER_AND_STILL_MORE_VERBOSE_DESCRIPTION_OF_AN_ENUMERATION_STATE,
           THE_FOLLOWING_IS_TAKEN_FROM_RAIBERT_1986_ONE_PART_OF_THE_CONTROL_SYSTEM_EXCITED_THE_CYCLIC_MOTION_THAT_UNDERLIES_RUNNING_WHILE_REGULATING_THE_HEIGHT_TO_WHICH_THE_MACHINE_HOPPED}

   public enum SmallEnum
   {
      A, B, C, D, F, G, EX, NINE, IF, NOR, FINE
   }

   private class SimpleController implements RobotControllerWithAttachRobot
   {
      private YoVariableRegistry registry;
      private EnumYoVariable<BadGreekEnum> badGreekVariable;
      private EnumYoVariable<LargeEnum> largeEnumVariable;
      private EnumYoVariable<SmallEnum> smallEnumVariable;
      private DoubleYoVariable numberVariable;
      private DoubleYoVariable time;
      private final String name = "simpleController";

      public SimpleController()
      {
      }

      @Override
      public void attachRobot(Robot robot)
      {
         registry = new YoVariableRegistry("controllerRegistry");
         badGreekVariable = new EnumYoVariable<BadGreekEnum>("badGreekVariable", registry, BadGreekEnum.class);
         badGreekVariable.set(BadGreekEnum.ALPHA);
         largeEnumVariable = new EnumYoVariable<LargeEnum>("largeEnumVariable", registry, LargeEnum.class);
         largeEnumVariable
            .set(LargeEnum
               .THE_FOLLOWING_IS_TAKEN_FROM_RAIBERT_1986_ONE_PART_OF_THE_CONTROL_SYSTEM_EXCITED_THE_CYCLIC_MOTION_THAT_UNDERLIES_RUNNING_WHILE_REGULATING_THE_HEIGHT_TO_WHICH_THE_MACHINE_HOPPED);
         smallEnumVariable = new EnumYoVariable<SmallEnum>("smallEnumVariable", registry, SmallEnum.class);
         smallEnumVariable.set(SmallEnum.IF);
         numberVariable = new DoubleYoVariable("numberVariable", registry);
         numberVariable.set(42.0);
         time = robot.getYoTime();
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return name;
      }

      @Override
      public String getDescription()
      {
         return name;
      }

      @Override
      public void doControl()
      {
         switch (badGreekVariable.getEnumValue())
         {
            case ALPHA :

               break;

            case BETA :
               break;

            case GAMMA :
               break;

            default :
               break;
         }

      }

      public DoubleYoVariable getTimeVariable()
      {
         return time;
      }

      public EnumYoVariable<BadGreekEnum> getBadGreekVariable()
      {
         return badGreekVariable;
      }

      public DoubleYoVariable getNumberVariable()
      {
         return numberVariable;
      }
   }


   public final static int DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING = 360;    // 400 and you can see it change. lower than 100 and scs isn't fast enough for the asserts.
   public final static int DELAY_TIME_FOR_TESTING_CONCURRENT_UPDATE = 1000;    // TODO: Find a way to avoid having this parameter matter. I worry about this one.

   // GT - these values are twice the minimum working values for Trogdor.
   // This test is for humans to view
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testEnumDisplay() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      SimpleController controller = new SimpleController();
      SimulationConstructionSet scs = setupSCS(controller);
      scs.setupEntryBox("largeEnumVariable");
      scs.setupEntryBox("badGreekVariable");
      scs.setupEntryBox("smallEnumVariable");
      Thread.sleep(10000);
      
      scs.closeAndDispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 7.4)
	@Test(timeout = 37000)
   public void testSwitchToEnumEntry() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      SimpleController controller = new SimpleController();
      SimulationConstructionSet scs = setupSCS(controller);
      StandardSimulationGUI scsGUI = scs.getStandardSimulationGUI();

      // first entry box is time
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      scs.setupEntryBox("t");
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      scs.setupEntryBox("q_x");
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      scs.setupEntryBox("badGreekVariable");
      ArrayList<YoEntryBox> entryBoxes = scsGUI.getEntryBoxArrayPanel().getEntryBoxesOnThisPanel();

      EnumYoVariable<BadGreekEnum> badGreekVariable = controller.getBadGreekVariable();
      DoubleYoVariable numberVariable = controller.getNumberVariable();
      DoubleYoVariable timeVariable = controller.getTimeVariable();
      badGreekVariable.set(BadGreekEnum.BETA);
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      assertEquals(timeVariable, entryBoxes.get(0).getVariableInThisBox());
      assertEquals(scs.getVariable("q_x"), entryBoxes.get(1).getVariableInThisBox());
      assertEquals(badGreekVariable, entryBoxes.get(2).getVariableInThisBox());
      assertEquals(YoTextEntryContainer.class, entryBoxes.get(1).getActiveYoVariableEntryContainer().getClass());
      assertEquals(YoEnumEntryContainer.class, entryBoxes.get(2).getActiveYoVariableEntryContainer().getClass());
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);

      YoEntryBox badGreekBox = entryBoxes.get(2);
      badGreekBox.removeVariable(badGreekBox.getVariableInThisBox());
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      assertEquals(YoTextEntryContainer.class, entryBoxes.get(2).getActiveYoVariableEntryContainer().getClass());

      YoEntryBox oldNumericVariableBox = entryBoxes.get(1);
      YoEntryBox newBadGreekBox = oldNumericVariableBox;
      newBadGreekBox.setVariableInThisBox(badGreekVariable);
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      badGreekVariable.set(BadGreekEnum.GAMMA);
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      badGreekVariable.set(BadGreekEnum.ALPHA);
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);

      assertEquals(YoEnumEntryContainer.class, newBadGreekBox.getActiveYoVariableEntryContainer().getClass());
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING * 4);

      scs.closeAndDispose();
   }

   public <T extends RobotControllerWithAttachRobot> SimulationConstructionSet setupSCS(T controller) throws InterruptedException
   {
      boolean showGUI = true;
      FallingBrickRobot robot = new FallingBrickRobot();
      controller.attachRobot(robot);
      assertFalse(controller.getYoVariableRegistry() == null);
      robot.setController(controller);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(2000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setFrameMaximized();
      scs.startOnAThread();
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      StandardSimulationGUI scsGUI = scs.getStandardSimulationGUI();
      EntryBoxArrayPanel temp = scsGUI.getEntryBoxArrayPanel();
      temp.removeAllEntryBoxes();
      scsGUI.getEntryBoxArrayPanel().updateUI();
      Thread.sleep(DELAY_TIME_FOR_HUMAN_CONVENIENT_VIEWING);
      return scs;
   }

   public interface RobotControllerWithAttachRobot extends RobotController
   {
      public void attachRobot(Robot robotToAttach);
   }
}
