package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class LocalYoWhiteBoard extends YoWhiteBoard
{
   private static final boolean VERBOSE = false;
   private YoWhiteBoard myBrotherWhiteBoard;

   public LocalYoWhiteBoard(String name, YoVariableRegistry registry)
   {
      super(name, registry);
   }

   public void setMyBrotherWhiteBoard(LocalYoWhiteBoard myBrotherWhiteBoard)
   {
      if (this.myBrotherWhiteBoard != null)
         throw new RuntimeException("myBrotherWhiteBoard != null");
      
      if (myBrotherWhiteBoard == this)
         throw new RuntimeException("myBrotherWhiteBoard == this");

      this.myBrotherWhiteBoard = myBrotherWhiteBoard;
      if (myBrotherWhiteBoard.myBrotherWhiteBoard != this)
         myBrotherWhiteBoard.setMyBrotherWhiteBoard(this);
   }

   @Override
   public void whiteBoardSpecificConnect()
   {
      ArrayList<YoVariable<?>> allVariablesToRead = new ArrayList<YoVariable<?>>();
      getAllVariablesToRead(allVariablesToRead);

      ArrayList<YoVariable<?>> brothersVariablesToWrite = new ArrayList<YoVariable<?>>();
      myBrotherWhiteBoard.getAllVariablesToWrite(brothersVariablesToWrite);

      verifyYoVariablesHaveSameNamesAndTypes(allVariablesToRead, brothersVariablesToWrite);


      ArrayList<YoVariable<?>> allVariablesToWrite = new ArrayList<YoVariable<?>>();
      getAllVariablesToWrite(allVariablesToWrite);

      ArrayList<YoVariable<?>> brothersVariablesToRead = new ArrayList<YoVariable<?>>();
      myBrotherWhiteBoard.getAllVariablesToRead(brothersVariablesToRead);

      verifyYoVariablesHaveSameNamesAndTypes(allVariablesToWrite, brothersVariablesToRead);
      
      if (VERBOSE)
         System.out.println("DataStreamYoWhiteBoard: Created " + allVariablesToRead.size() + " variablesToRead and " + allVariablesToWrite.size() + " variablesToWrite");


      setConnected(true);
   }

   @Override
   public void whiteBoardSpecificWriteData(double[] doubleVariablesToWriteBuffer, int[] intVariablesToWriteBuffer, boolean[] booleanVariablesToWriteBuffer,
           int[] enumVariablesToWriteBuffer, int writeIndex)
   {
      myBrotherWhiteBoard.setVariablesToReadBuffers(doubleVariablesToWriteBuffer, intVariablesToWriteBuffer, booleanVariablesToWriteBuffer,
              enumVariablesToWriteBuffer, writeIndex);
   }


   @Override
   public void closeYoWhiteBoard() throws IOException
   {
      setConnected(false);       
   }

   private void verifyYoVariablesHaveSameNamesAndTypes(ArrayList<YoVariable<?>> variablesOne, ArrayList<YoVariable<?>> variablesTwo)
   {
      if (variablesOne.size() != variablesTwo.size())
      {
         throw new RuntimeException("variablesOne.size() != variablesTwo.size()");
      }
   
      for (int i = 0; i < variablesOne.size(); i++)
      {
         YoVariable<?> variableOne = variablesOne.get(i);
         YoVariable<?> variableTwo = variablesTwo.get(i);
         verifyTypesAreEqual(variableOne, variableTwo);
         verifyNamesAreConsistent(variableOne.getFullNameWithNameSpace(), variableTwo.getFullNameWithNameSpace());
      }
   }

   private void verifyTypesAreEqual(YoVariable<?> variableOne, YoVariable<?> variableTwo)
   {
      if (variableOne.getYoVariableType() != variableTwo.getYoVariableType())
         throw new RuntimeException("variableOne.getYoVariableType() != variableTwo.getYoVariableType()");
   }
}
