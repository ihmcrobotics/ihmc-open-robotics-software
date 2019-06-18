package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import static us.ihmc.robotics.Assert.*;

import java.io.IOException;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.util.NetworkPorts;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoactiveElementYoWhiteBoardSynchronizerTest
{
   @Test
   public void testCoactiveElementTCPYoWhiteBoardSynchronizer() throws IOException, InterruptedException
   {
      int port = NetworkPorts.COACTIVE_ELEMENTS_PORT.getPort();

      LogTools.info("Creating human");
      SimpleCoactiveElement userInterfaceSideCoactiveElement = new SimpleCoactiveElement();
      userInterfaceSideCoactiveElement.initializeUserInterfaceSide();
      CoactiveElementYoWhiteBoardSynchronizer humanSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(port, HumanOrMachine.HUMAN, userInterfaceSideCoactiveElement);
      assertEquals(HumanOrMachine.HUMAN, humanSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(userInterfaceSideCoactiveElement, humanSideSynchronizer.getCoactiveElement());

      LogTools.info("Starting human");
      humanSideSynchronizer.startASynchronizerOnAThread(100L);
      while (port == 0)
      {
         port = humanSideSynchronizer.getYoWhiteBoard().getPort();
         Thread.yield();
      }

      LogTools.info("creating machine");
      
      SimpleCoactiveElement machineSideCoactiveElement = new SimpleCoactiveElement();
      machineSideCoactiveElement.initializeMachineSide();
      CoactiveElementYoWhiteBoardSynchronizer machineSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(port, "localHost", HumanOrMachine.MACHINE, machineSideCoactiveElement);
      assertEquals(HumanOrMachine.MACHINE, machineSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(machineSideCoactiveElement, machineSideSynchronizer.getCoactiveElement());

      LogTools.info("Starting machine");
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
      private final YoInteger uiTickCount = new YoInteger("uiTickCount", userInterfaceWritableRegistry);
      private final YoBoolean buttonWasClicked = new YoBoolean("buttonWasClicked", userInterfaceWritableRegistry);
      private final YoDouble variableForUserInterfaceToWrite = new YoDouble("variableForUserInterfaceToWrite", userInterfaceWritableRegistry);

      private final YoVariableRegistry machineWritableRegistry = new YoVariableRegistry("machineWritableRegistry");
      private final YoDouble variableForMachineToWrite = new YoDouble("variableForMachineToWrite", machineWritableRegistry);

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
               variableForUserInterfaceToWrite.set(RandomNumbers.nextDouble(random, 20.0));
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
            variableForMachineToWrite.set(RandomNumbers.nextDouble(random, variableForUserInterfaceToWrite.getDoubleValue()));
         }
         else
         {
            variableForMachineToWrite.set(RandomNumbers.nextDouble(random, 1.0));
         }
      }

      @Override
      public YoVariableRegistry getMachineWritableYoVariableRegistry()
      {
         return machineWritableRegistry;
      }
   }
}
