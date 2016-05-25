package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class CoactiveElementYoWhiteBoardSynchronizer
{
   private final TCPYoWhiteBoard tcpYoWhiteBoard;

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
         this.tcpYoWhiteBoard = new TCPYoWhiteBoard(whichSideIsThisRunningOn + "SideCoactiveElementYoWhiteBoard", port);
      }
      else
      {
         this.tcpYoWhiteBoard = new TCPYoWhiteBoard(whichSideIsThisRunningOn + "SideCoactiveElementYoWhiteBoard", ipAddress, port);
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

      tcpYoWhiteBoard.setVariablesToRead(variablesToRead);
      tcpYoWhiteBoard.setVariablesToWrite(variablesToWrite);
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
      tcpYoWhiteBoard.writeData();
   }

   public boolean isNewDataAvailable()
   {
      return tcpYoWhiteBoard.isNewDataAvailable();
   }

   public void readData() throws IOException
   {
      tcpYoWhiteBoard.readData();
   }

   public void startASynchronizerOnAThread(long millisecondsBetweenDataWrites)
   {
      tcpYoWhiteBoard.startOnAThread();
      
      SynchronizerRunnable synchronizerRunnable = new SynchronizerRunnable(this, millisecondsBetweenDataWrites);
      ThreadTools.startAThread(synchronizerRunnable, getClass().getSimpleName());
   }
   
   public void close()
   {
      try
      {
         PrintTools.info(this, "Closed.");
         tcpYoWhiteBoard.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private class SynchronizerRunnable implements Runnable
   {
      private static final boolean DEBUG = false;

      private final CoactiveElementYoWhiteBoardSynchronizer coactiveElementYoWhiteBoardSynchronizer;
      private final long millisecondsBetweenDataWrites;

      SynchronizerRunnable(CoactiveElementYoWhiteBoardSynchronizer coactiveElementYoWhiteBoardSynchronizer, long millisecondsBetweenDataWrites)
      {
         this.coactiveElementYoWhiteBoardSynchronizer = coactiveElementYoWhiteBoardSynchronizer;
         this.millisecondsBetweenDataWrites = millisecondsBetweenDataWrites;
      }

      @Override
      public void run()
      {
         while (true)
         {
            try
            {
               PrintTools.debug(DEBUG, this, "coactiveElementYoWhiteBoardSynchronizer.writeData()");
               coactiveElementYoWhiteBoardSynchronizer.writeData();
               {
                  PrintTools.debug(DEBUG, this, "coactiveElementYoWhiteBoardSynchronizer.readData()");
                  coactiveElementYoWhiteBoardSynchronizer.readData();
               }
            }
            catch (Exception e)
            {
               PrintTools.debug(DEBUG, this, e.getMessage());
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
      return tcpYoWhiteBoard;
   }

}
