package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;


import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateChangeRecorder;

public class StateChangeRecorderTest
{
   private ArrayList<State<TestEnum>> listOfStates;
   private StateChangeRecorder<TestEnum> stateChangeRecorder;

   public StateChangeRecorderTest()
   {
   }

   @BeforeEach
   public void setUp() throws Exception
   {
      listOfStates = getListOfStates();
      stateChangeRecorder = new StateChangeRecorder<TestEnum>();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
   public void testConstructor()
   {
      // empty, work being done in setUp()
   }

	@Test
   public void testAddingValues()
   {
      int index = Math.max(0, listOfStates.size() - 2);
      State<TestEnum> stateToTest = listOfStates.get(index);

      ArrayList<Double> listOfTimes = new ArrayList<Double>();
      listOfTimes.add(0.1345);
      listOfTimes.add(1.567);
      listOfTimes.add(4.8653456);
      listOfTimes.add(101223432.2324234);

      for (Double value : listOfTimes)
      {
         stateChangeRecorder.stateChanged(stateToTest, stateToTest, value);
      }

      ArrayList<Double> listOfSwitchTimes = stateChangeRecorder.getStatesAndSwitchTimes().get(stateToTest);
      assertEquals(listOfSwitchTimes.size(), listOfTimes.size());

      for (int i = 0; i < listOfSwitchTimes.size(); i++)
      {
         assertEquals(listOfSwitchTimes.get(i), listOfTimes.get(i));
      }
   }

	@Test
   public void testClearAllData()
   {
      int index = Math.max(0, listOfStates.size() - 2);
      State<TestEnum> stateToTest = listOfStates.get(index);


      ArrayList<Double> listOfTimes = new ArrayList<Double>();
      listOfTimes.add(0.1345);
      listOfTimes.add(1.567);
      listOfTimes.add(4.8653456);
      listOfTimes.add(10.2324234);

      for (Double value : listOfTimes)
      {
         stateChangeRecorder.stateChanged(stateToTest, stateToTest, value);
      }

      stateChangeRecorder.clearAllData();


      for (State<TestEnum> state : listOfStates)
      {
         ArrayList<Double> listOfSwitchTimes = stateChangeRecorder.getStatesAndSwitchTimes().get(state);
         if (listOfSwitchTimes != null && listOfSwitchTimes.size() > 0)
         {
            fail();
         }
      }
   }

   private ArrayList<State<TestEnum>> getListOfStates()
   {
      ArrayList<State<TestEnum>> listOfStates = new ArrayList<State<TestEnum>>();

      for (TestEnum enumValue : TestEnum.values())
      {
         State<TestEnum> state = new DummyState(enumValue);
         listOfStates.add(state);
      }

      return listOfStates;
   }

   private class DummyState extends State<TestEnum>
   {
      public DummyState(TestEnum stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doAction()
      {
         // empty
      }

      @Override
      public void doTransitionIntoAction()
      {
         // empty
      }

      @Override
      public void doTransitionOutOfAction()
      {
         // empty
      }

   }


   private enum TestEnum
   {
      HIP_PITCH, HIP_ROLL, HIP_YAW, KNEE, ANKLE_ROLL, ANKLE_PITCH;
   }

}
