package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;
import us.ihmc.tools.thread.ThreadTools;

public class CoactiveElementYoWhiteBoardSynchronizer
{
   private final TCPYoWhiteBoard yoWhiteBoard;

   private final HumanOrMachine whichSideIsThisRunningOn;
   private final CoactiveElement coactiveElement;

   private final YoVariableRegistry machineWritableYoVariableRegistry;
   private final YoVariableRegistry userInterfaceWritableYoVariableRegistry;

   /**
    * Server constructor.
    */
   public CoactiveElementYoWhiteBoardSynchronizer(int port, HumanOrMachine whichSideIsThisRunningOn, CoactiveElement coactiveElement)
   {
      this(port, null, whichSideIsThisRunningOn, coactiveElement);
   }
   
   /**
    * Client constructor.
    */
   public CoactiveElementYoWhiteBoardSynchronizer(int port, String ipAddress, HumanOrMachine whichSideIsThisRunningOn, CoactiveElement coactiveElement)
   {
      this.whichSideIsThisRunningOn = whichSideIsThisRunningOn;
      this.coactiveElement = coactiveElement;

      this.machineWritableYoVariableRegistry = coactiveElement.getMachineWritableYoVariableRegistry();
      this.userInterfaceWritableYoVariableRegistry = coactiveElement.getUserInterfaceWritableYoVariableRegistry();

      ArrayList<YoVariable<?>> variablesToRead = null;
      ArrayList<YoVariable<?>> variablesToWrite = null;

      if (ipAddress == null)
      {
         this.yoWhiteBoard = new TCPYoWhiteBoard(whichSideIsThisRunningOn + "SideCoactiveElementYoWhiteBoard", port);
      }
      else
      {
         this.yoWhiteBoard = new TCPYoWhiteBoard(whichSideIsThisRunningOn + "SideCoactiveElementYoWhiteBoard", ipAddress, port);
      }
      
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
      yoWhiteBoard.startOnAThread();
      
      SynchronizerRunnable synchronizerRunnable = new SynchronizerRunnable(this, millisecondsBetweenDataWrites);
      ThreadTools.startAThread(synchronizerRunnable, getClass().getSimpleName());
   }
   
   public void close()
   {
      try
      {
         yoWhiteBoard.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
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

   public TCPYoWhiteBoard getYoWhiteBoard()
   {
      return yoWhiteBoard;
   }

}
