package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.IOException;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class YoWhiteBoardViewerGUI
{
   private final UDPYoWhiteBoard whiteBoard;
   
   public YoWhiteBoardViewerGUI(String ipAddress, int sendPort, int receivePort) throws IOException
   {
      boolean runThisOneFirst = true;
      boolean createYoVariablesOnConnect = true;
      
      Robot robot = new Robot("YoWhiteBoardViewerGUI");
      
      YoVariableRegistry registry = new YoVariableRegistry("YoWhiteBoardViewer");
            
      whiteBoard = new UDPYoWhiteBoard("whiteBoardViewer", runThisOneFirst, ipAddress, sendPort, receivePort, false, createYoVariablesOnConnect, registry);
            
      whiteBoard.startUDPThread();
      
      while(!whiteBoard.haveVariablesToReadAndWriteBeenSet())
      {
         try
         {
            Thread.sleep(1000);
         } 
         catch (InterruptedException e)
         {
         }
      }
      
      whiteBoard.connect();

      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoVariableRegistry(registry);
      
//      scs.addVarList(registry.createVarList());
      
      Thread scsThread = new Thread(scs);
      scsThread.start();
      
      
      while(!whiteBoard.isConnected())
      {
         System.out.println("ViewerGUI waiting to connect");
         try
         {
            Thread.sleep(2000);
         } 
         catch (InterruptedException e)
         {
         }
      }
      
      System.out.println("ViewerGUI is connected. Writing and reading data in a loop.");

      while(true)
      {
         whiteBoard.writeData();
         
         while(!whiteBoard.isNewDataAvailable())
         {
            Thread.yield();
         }
         
         whiteBoard.readData();
         scs.tickAndUpdate();

//         try
//         {
//            Thread.sleep(100);
////            Thread.yield();
//         } 
//         catch (InterruptedException e)
//         {
//         }
         
      }
   }
   
   
   public static void main(String[] args) throws IOException
   {
      if ((args == null) || (args.length < 3))
      {
         throw new RuntimeException("Arguments must be ipAddress sendPort receivePort");
      }
//      String ipAddress = "cheetah.ihmc.us";
//      int sendPort = 7777;
//      int receivePort = 7777;

      String ipAddress = args[0];
      int sendPort = Integer.parseInt(args[1]);
      int receivePort = Integer.parseInt(args[2]);

      new YoWhiteBoardViewerGUI(ipAddress, sendPort, receivePort);
   }
}
