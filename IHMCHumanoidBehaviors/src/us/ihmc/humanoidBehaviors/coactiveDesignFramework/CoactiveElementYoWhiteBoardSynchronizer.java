package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class CoactiveElementYoWhiteBoardSynchronizer implements Runnable
{
   private static final boolean DEBUG = false;
   private long millisecondsBetweenDataWrites = 300L;
   private boolean closed = true;
   
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
      PrintTools.debug(DEBUG, this, "writeData()");
      tcpYoWhiteBoard.writeData();
   }

   public boolean isNewDataAvailable()
   {
      return tcpYoWhiteBoard.isNewDataAvailable();
   }

   public void readData() throws IOException
   {
      PrintTools.debug(DEBUG, this, "readData()");
      tcpYoWhiteBoard.readData();
   }

   public void startASynchronizerOnAThread(long millisecondsBetweenDataWrites)
   {
      if (!closed)
         throw new RuntimeException("Synchronizer already started");
      
      closed = false;
      
      this.millisecondsBetweenDataWrites = millisecondsBetweenDataWrites;
      
      tcpYoWhiteBoard.startTCPThread();
      startSynchronizerThread();
   }
   
   private void startSynchronizerThread()
   {
      ThreadTools.startAThread(this, getClass().getSimpleName());
   }

   @Override
   public void run()
   {
      while (!closed)
      {
         try
         {
            writeData();
            readData();
         }
         catch (Exception e)
         {
            PrintTools.error(this, e.getMessage());
            try
            {
               PrintTools.info(this, "Connecting");
               tcpYoWhiteBoard.connect();
            }
            catch (IOException ioException)
            {
               PrintTools.error(this, "Failed to connect");
               ioException.printStackTrace();
            }
            ThreadTools.sleep(500);
         }

         ThreadTools.sleep(millisecondsBetweenDataWrites);
      }
   }

   public void close()
   {
      try
      {
         PrintTools.info(this, "Closed.");
         closed = true;
         tcpYoWhiteBoard.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public TCPYoWhiteBoard getYoWhiteBoard()
   {
      return tcpYoWhiteBoard;
   }
}
