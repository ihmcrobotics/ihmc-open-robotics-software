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

}
