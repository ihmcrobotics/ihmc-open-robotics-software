package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;

public class CoactiveBehaviorTools
{
   public static final int PORT_FOR_COACTIVE_ELEMENTS = 56122;

   public static void synchronizeCoactiveElementMachineSideUsingTCPYoWhiteBoard(CoactiveElement coactiveElement)
   {
      if (coactiveElement != null)
      {

         YoVariableRegistry userInterfaceWritableYoVariableRegistry = coactiveElement.getUserInterfaceWritableYoVariableRegistry();
         YoVariableRegistry machineWritableYoVariableRegistry = coactiveElement.getMachineWritableYoVariableRegistry();

         HumanOrMachine whichSideIsThisRunningOn = HumanOrMachine.MACHINE;
         TCPYoWhiteBoard yoWhiteBoard = new TCPYoWhiteBoard("CoactiveElementYoWhiteBoard", PORT_FOR_COACTIVE_ELEMENTS);
         yoWhiteBoard.setVariablesToRead(userInterfaceWritableYoVariableRegistry.getAllVariablesIncludingDescendants());
         yoWhiteBoard.setVariablesToWrite(machineWritableYoVariableRegistry.getAllVariablesIncludingDescendants());
         
         CoactiveElementYoWhiteBoardSynchronizer synchronizer = new CoactiveElementYoWhiteBoardSynchronizer(yoWhiteBoard, whichSideIsThisRunningOn, coactiveElement);

         Thread thread = new Thread(yoWhiteBoard);
         thread.start();

         try
         {
            System.out.println("Connecting to YoWhiteBoard");
            yoWhiteBoard.connect();
         }
         catch (IOException e)
         {
            System.err.println("Could not connect to YoWhiteBoard");
            e.printStackTrace();
         }
         
         long millisecondsBetweenDataWrites = 100L;
         synchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);
      }
   }
}
