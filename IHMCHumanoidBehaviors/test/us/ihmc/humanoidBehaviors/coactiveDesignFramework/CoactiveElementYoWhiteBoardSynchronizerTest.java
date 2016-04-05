package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import static org.junit.Assert.*;

import java.io.IOException;
import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class CoactiveElementYoWhiteBoardSynchronizerTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCoactiveElementTCPYoWhiteBoardSynchronizer() throws IOException, InterruptedException
   {
      int port = 0;

      SimpleCoactiveElement userInterfaceSideCoactiveElement = new SimpleCoactiveElement();
      userInterfaceSideCoactiveElement.initializeUserInterfaceSide();
      TCPYoWhiteBoard userInterfaceSideWhiteBoard = new TCPYoWhiteBoard("UserInterfaceSideWhiteBoard", port);
      CoactiveElementYoWhiteBoardSynchronizer userInterfaceSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(userInterfaceSideWhiteBoard, HumanOrMachine.HUMAN, userInterfaceSideCoactiveElement);
      assertEquals(HumanOrMachine.HUMAN, userInterfaceSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(userInterfaceSideCoactiveElement, userInterfaceSideSynchronizer.getCoactiveElement());

      Thread userInterfaceSideThread = new Thread(userInterfaceSideWhiteBoard);
      userInterfaceSideThread.start();

      while (port == 0)
      {
         port = userInterfaceSideWhiteBoard.getPort();
         Thread.yield();
      }

      SimpleCoactiveElement machineSideCoactiveElement = new SimpleCoactiveElement();
      machineSideCoactiveElement.initializeMachineSide();
      TCPYoWhiteBoard machineSideWhiteBoard = new TCPYoWhiteBoard("MachineSideWhiteBoard", "localHost", port);
      CoactiveElementYoWhiteBoardSynchronizer machineSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(machineSideWhiteBoard, HumanOrMachine.MACHINE, machineSideCoactiveElement);
      assertEquals(HumanOrMachine.MACHINE, machineSideSynchronizer.getWhichSideIsThisRunningOn());
      assertEquals(machineSideCoactiveElement, machineSideSynchronizer.getCoactiveElement());

      Thread machineSideThread = new Thread(machineSideWhiteBoard);
      machineSideThread.start();

      userInterfaceSideWhiteBoard.connect();
      machineSideWhiteBoard.connect();
      
      while(!userInterfaceSideWhiteBoard.isConnected())
      {
         Thread.yield();
      }
      
      while(!machineSideWhiteBoard.isConnected())
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
         userInterfaceSideSynchronizer.writeData();

         while (!machineSideSynchronizer.isNewDataAvailable())
         {
            Thread.yield();
         }
         while (!userInterfaceSideSynchronizer.isNewDataAvailable())
         {
            Thread.yield();
         }

         machineSideSynchronizer.readData();
         userInterfaceSideSynchronizer.readData();

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
      userInterfaceSideWhiteBoard.close();
      machineSideWhiteBoard.close();
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
