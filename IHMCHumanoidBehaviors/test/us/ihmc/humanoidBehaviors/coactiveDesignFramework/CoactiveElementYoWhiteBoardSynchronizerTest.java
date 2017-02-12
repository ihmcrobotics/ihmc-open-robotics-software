package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.Random;

import org.junit.Test;

import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.random.RandomTools;

public class CoactiveElementYoWhiteBoardSynchronizerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCoactiveElementTCPYoWhiteBoardSynchronizer() throws IOException, InterruptedException
   {
      int port = NetworkPorts.COACTIVE_ELEMENTS_PORT.getPort();

      System.out.println("Creating human");
      SimpleCoactiveElement userInterfaceSideCoactiveElement = new SimpleCoactiveElement();
      userInterfaceSideCoactiveElement.initializeUserInterfaceSide();
      CoactiveElementYoWhiteBoardSynchronizer humanSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(port, HumanOrMachine.HUMAN, userInterfaceSideCoactiveElement);
      assertEquals(HumanOrMachine.HUMAN, humanSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(userInterfaceSideCoactiveElement, humanSideSynchronizer.getCoactiveElement());

      System.out.println("Starting human");
      humanSideSynchronizer.startASynchronizerOnAThread(100L);
      while (port == 0)
      {
         port = humanSideSynchronizer.getYoWhiteBoard().getPort();
         Thread.yield();
      }

      System.out.println("creating machine");
      
      SimpleCoactiveElement machineSideCoactiveElement = new SimpleCoactiveElement();
      machineSideCoactiveElement.initializeMachineSide();
      CoactiveElementYoWhiteBoardSynchronizer machineSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(port, "localHost", HumanOrMachine.MACHINE, machineSideCoactiveElement);
      assertEquals(HumanOrMachine.MACHINE, machineSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(machineSideCoactiveElement, machineSideSynchronizer.getCoactiveElement());

      System.out.println("Starting machine");
      machineSideSynchronizer.startASynchronizerOnAThread(100L);
      
      while(!humanSideSynchronizer.getYoWhiteBoard().isConnected())
      {
         Thread.yield();
      }
      
      while(!machineSideSynchronizer.getYoWhiteBoard().isConnected())
      {
         Thread.yield();
      }

      boolean buttonWasClickedAtLeastOnce = false;
      int numberOfTicksToTest = 100;
      for (int tickNumber = 1; tickNumber < numberOfTicksToTest; tickNumber++)
      {
         userInterfaceSideCoactiveElement.updateUserInterfaceSide();
         machineSideCoactiveElement.updateMachineSide();

         assertNotEquals(userInterfaceSideCoactiveElement.getUITickCount(), machineSideCoactiveElement.getUITickCount());
         assertNotEquals(userInterfaceSideCoactiveElement.getVariableForMachineToWrite(), machineSideCoactiveElement.getVariableForMachineToWrite(), 1e-7);

         machineSideSynchronizer.writeData();
         humanSideSynchronizer.writeData();

         while (!machineSideSynchronizer.isNewDataAvailable())
         {
            Thread.yield();
         }
         while (!humanSideSynchronizer.isNewDataAvailable())
         {
            Thread.yield();
         }

         machineSideSynchronizer.readData();
         humanSideSynchronizer.readData();

         assertEquals(tickNumber, userInterfaceSideCoactiveElement.getUITickCount());
         assertEquals(userInterfaceSideCoactiveElement.getUITickCount(), machineSideCoactiveElement.getUITickCount());
         assertEquals(userInterfaceSideCoactiveElement.getButtonWasClicked(), machineSideCoactiveElement.getButtonWasClicked());
         assertEquals(userInterfaceSideCoactiveElement.getVariableForUserInterfaceToWrite(), machineSideCoactiveElement.getVariableForUserInterfaceToWrite(), 1e-7);
         assertEquals(userInterfaceSideCoactiveElement.getVariableForMachineToWrite(), machineSideCoactiveElement.getVariableForMachineToWrite(), 1e-7);

         if (userInterfaceSideCoactiveElement.getButtonWasClicked())
         {
            buttonWasClickedAtLeastOnce = true;
         }
      }

      assertTrue(buttonWasClickedAtLeastOnce);
      humanSideSynchronizer.close();
      machineSideSynchronizer.close();
   }

   private class SimpleCoactiveElement implements CoactiveElement
   {
      private final YoVariableRegistry userInterfaceWritableRegistry = new YoVariableRegistry("userInterfaceWritable");
      private final IntegerYoVariable uiTickCount = new IntegerYoVariable("uiTickCount", userInterfaceWritableRegistry);
      private final BooleanYoVariable buttonWasClicked = new BooleanYoVariable("buttonWasClicked", userInterfaceWritableRegistry);
      private final DoubleYoVariable variableForUserInterfaceToWrite = new DoubleYoVariable("variableForUserInterfaceToWrite", userInterfaceWritableRegistry);

      private final YoVariableRegistry machineWritableRegistry = new YoVariableRegistry("machineWritableRegistry");
      private final DoubleYoVariable variableForMachineToWrite = new DoubleYoVariable("variableForMachineToWrite", machineWritableRegistry);

      private final Random random = new Random();

      public SimpleCoactiveElement()
      {
      }

      public int getUITickCount()
      {
         return uiTickCount.getIntegerValue();
      }

      public boolean getButtonWasClicked()
      {
         return buttonWasClicked.getBooleanValue();
      }

      public double getVariableForUserInterfaceToWrite()
      {
         return variableForUserInterfaceToWrite.getDoubleValue();
      }

      public double getVariableForMachineToWrite()
      {
         return variableForMachineToWrite.getDoubleValue();
      }

      @Override
      public void initializeUserInterfaceSide()
      {
      }

      @Override
      public void updateUserInterfaceSide()
      {
         uiTickCount.increment();

         // Simulate a button being clicked on and off every so often. In a real situation, this will be done through a GUI button.
         if (uiTickCount.getIntegerValue() % 5 == 0)
         {
            buttonWasClicked.set(!buttonWasClicked.getBooleanValue());

            if (buttonWasClicked.getBooleanValue())
            {
               variableForUserInterfaceToWrite.set(RandomTools.generateRandomDouble(random, 20.0));
            }
         }
      }

      @Override
      public YoVariableRegistry getUserInterfaceWritableYoVariableRegistry()
      {
         return userInterfaceWritableRegistry;
      }

      @Override
      public void initializeMachineSide()
      {
      }

      @Override
      public void updateMachineSide()
      {
         if (buttonWasClicked.getBooleanValue())
         {
            variableForMachineToWrite.set(RandomTools.generateRandomDouble(random, variableForUserInterfaceToWrite.getDoubleValue()));
         }
         else
         {
            variableForMachineToWrite.set(RandomTools.generateRandomDouble(random, 1.0));
         }
      }

      @Override
      public YoVariableRegistry getMachineWritableYoVariableRegistry()
      {
         return machineWritableRegistry;
      }
   }
}
