package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.whiteBoard.YoWhiteBoard;

public class CoactiveElementYoWhiteBoardSynchronizer
{
   private final YoWhiteBoard yoWhiteBoard;

   private final HumanOrMachine whichSideIsThisRunningOn;
   private final CoactiveElement coactiveElement;

   private final YoVariableRegistry machineWritableYoVariableRegistry;
   private final YoVariableRegistry userInterfaceWritableYoVariableRegistry;

   public CoactiveElementYoWhiteBoardSynchronizer(YoWhiteBoard yoWhiteBoard, HumanOrMachine whichSideIsThisRunningOn, CoactiveElement coactiveElement)
   {
      this.yoWhiteBoard = yoWhiteBoard;

      this.whichSideIsThisRunningOn = whichSideIsThisRunningOn;
      this.coactiveElement = coactiveElement;

      this.machineWritableYoVariableRegistry = coactiveElement.getMachineWritableYoVariableRegistry();
      this.userInterfaceWritableYoVariableRegistry = coactiveElement.getUserInterfaceWritableYoVariableRegistry();

      ArrayList<YoVariable<?>> variablesToRead = null;
      ArrayList<YoVariable<?>> variablesToWrite = null;

      switch (whichSideIsThisRunningOn)
      {
      case MACHINE:
      {
         variablesToWrite = machineWritableYoVariableRegistry.getAllVariablesIncludingDescendants();
         variablesToRead = userInterfaceWritableYoVariableRegistry.getAllVariablesIncludingDescendants();
         break;
      }
      case HUMAN:
      {
         variablesToRead = machineWritableYoVariableRegistry.getAllVariablesIncludingDescendants();
         variablesToWrite = userInterfaceWritableYoVariableRegistry.getAllVariablesIncludingDescendants();
         break;
      }
      }

      yoWhiteBoard.setVariablesToRead(variablesToRead);
      yoWhiteBoard.setVariablesToWrite(variablesToWrite);
   }

   public CoactiveElement getCoactiveElement()
   {
      return coactiveElement;
   }

   public HumanOrMachine getWhichSideIsThisRunningOn()
   {
      return whichSideIsThisRunningOn;
   }

   public void writeData() throws IOException
   {
      yoWhiteBoard.writeData();
   }

   public boolean isNewDataAvailable()
   {
      return yoWhiteBoard.isNewDataAvailable();
   }

   public void readData() throws IOException
   {
      yoWhiteBoard.readData();
   }

   public void startASynchronizerOnAThread(long millisecondsBetweenDataWrites)
   {
      SynchronizerRunnable synchronizerRunnable = new SynchronizerRunnable(this, millisecondsBetweenDataWrites);
      Thread thread = new Thread(synchronizerRunnable);
      thread.start();
   }

   private class SynchronizerRunnable implements Runnable
   {
      private static final boolean DEBUG = false;

      private final CoactiveElementYoWhiteBoardSynchronizer synchronizer;
      private final long millisecondsBetweenDataWrites;

      SynchronizerRunnable(CoactiveElementYoWhiteBoardSynchronizer synchronizer, long millisecondsBetweenDataWrites)
      {
         this.synchronizer = synchronizer;
         this.millisecondsBetweenDataWrites = millisecondsBetweenDataWrites;
      }

      @Override
      public void run()
      {
         while (true)
         {
            try
            {
               if (DEBUG)
                  System.out.println("Writing Data");
               synchronizer.writeData();
               {
                  if (DEBUG)
                     System.out.println("Reading Data");
                  synchronizer.readData();
               }
            }
            catch (Exception e)
            {
               if (DEBUG)
                  System.out.println("Exception: " + e);
            }

            try
            {
               Thread.sleep(millisecondsBetweenDataWrites);
            }
            catch (InterruptedException e)
            {
            }
         }
      }
   }

}
