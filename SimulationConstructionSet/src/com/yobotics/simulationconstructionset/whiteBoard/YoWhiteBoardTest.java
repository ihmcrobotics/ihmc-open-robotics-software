package com.yobotics.simulationconstructionset.whiteBoard;


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.Test;

import com.yobotics.simulationconstructionset.AbstractYoVariable;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoWhiteBoardTest
{
   @Test
   public void testNothing()
   {
      // Just put one blank test here. The concrete versions of the classes, such as LocalYoWhiteBoardTest should run the real tests, 
      // using the helper methods below.
   }
   
   protected void doATest(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard) throws IOException
   {
      createVariablesAndAddToWhiteBoard(leftWhiteBoard, true);
      createVariablesAndAddToWhiteBoard(rightWhiteBoard, false);

      leftWhiteBoard.connect();
      rightWhiteBoard.connect();

      YoWhiteBoardListenerForTest leftWhiteBoardListener = new YoWhiteBoardListenerForTest();
      YoWhiteBoardListenerForTest rightWhiteBoardListener = new YoWhiteBoardListenerForTest();

      leftWhiteBoard.attachYoWhiteBoardListener(leftWhiteBoardListener);
      rightWhiteBoard.attachYoWhiteBoardListener(rightWhiteBoardListener);

      int numberOfTests = 1000;

      for (int i = 0; i < numberOfTests; i++)
      {
         changeWrittenVariablesRandomly(leftWhiteBoard);
         changeWrittenVariablesRandomly(rightWhiteBoard);

         assertFalse(leftWhiteBoardListener.hasReceivedNewDataBeforeReset());
         assertFalse(rightWhiteBoardListener.hasReceivedNewDataBeforeReset());

         leftWhiteBoardListener.reset();
         rightWhiteBoardListener.reset();

         leftWhiteBoard.writeData();
         rightWhiteBoard.writeData();

         while (!leftWhiteBoardListener.isNewDataReady() ||!rightWhiteBoardListener.isNewDataReady())
         {
            try
            {
               Thread.sleep(1);
            }
            catch (InterruptedException e)
            {
            }

         }

         leftWhiteBoard.readData();
         rightWhiteBoard.readData();

         verifyWhiteBoardsHaveSameData(leftWhiteBoard, rightWhiteBoard);
      }
   }

   private void verifyWhiteBoardsHaveSameData(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard)
   {
      ArrayList<AbstractYoVariable> leftVariablesToWrite = new ArrayList<AbstractYoVariable>();
      ArrayList<AbstractYoVariable> leftVariablesToRead = new ArrayList<AbstractYoVariable>();

      ArrayList<AbstractYoVariable> rightVariablesToWrite = new ArrayList<AbstractYoVariable>();
      ArrayList<AbstractYoVariable> rightVariablesToRead = new ArrayList<AbstractYoVariable>();


      leftWhiteBoard.getAllVariablesToWrite(leftVariablesToWrite);
      leftWhiteBoard.getAllVariablesToRead(leftVariablesToRead);

      rightWhiteBoard.getAllVariablesToWrite(rightVariablesToWrite);
      rightWhiteBoard.getAllVariablesToRead(rightVariablesToRead);



      verifyYoVariablesAreEqual(leftVariablesToWrite, rightVariablesToRead);
      verifyYoVariablesAreEqual(leftVariablesToRead, rightVariablesToWrite);

   }

   private void verifyYoVariablesAreEqual(ArrayList<AbstractYoVariable> variablesOne, ArrayList<AbstractYoVariable> variablesTwo)
   {
      assertEquals(variablesOne.size(), variablesTwo.size());

      for (int i = 0; i < variablesOne.size(); i++)
      {
         verifyYoVariablesAreEqual(variablesOne.get(i), variablesTwo.get(i));
      }

   }

   private void verifyYoVariablesAreEqual(AbstractYoVariable variableOne, AbstractYoVariable variableTwo)
   {
      assertTrue(variableOne.getYoVariableType() == variableTwo.getYoVariableType());
      assertTrue(variableOne.getFullNameWithNameSpace().equals(variableTwo.getFullNameWithNameSpace()));

      assertEquals(variableOne.getValueAsDouble(), variableTwo.getValueAsDouble(), 1e-7);
   }

   private class YoWhiteBoardListenerForTest implements YoWhiteBoardListener
   {
      private boolean newDataReady = false;
      private boolean hasReceivedNewDataBeforeReset = false;

      public void receivedWhiteBoardData()
      {
         if (newDataReady)
         {
            hasReceivedNewDataBeforeReset = true;
         }

         newDataReady = true;
      }

      public void reset()
      {
         newDataReady = false;
      }

      public boolean isNewDataReady()
      {
         return newDataReady;
      }

      public boolean hasReceivedNewDataBeforeReset()
      {
         return hasReceivedNewDataBeforeReset;
      }
   }


   private void changeWrittenVariablesRandomly(YoWhiteBoard leftWhiteBoard)
   {
      ArrayList<AbstractYoVariable> variablesToWrite = new ArrayList<AbstractYoVariable>();
      leftWhiteBoard.getAllVariablesToWrite(variablesToWrite);

      for (AbstractYoVariable variable : variablesToWrite)
      {
         variable.setValueFromDouble(2.0 * Math.random());
      }
   }

   private void createVariablesAndAddToWhiteBoard(YoWhiteBoard whiteBoard, boolean readOddWriteEven)
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("root");
      YoVariableRegistry registryOne = new YoVariableRegistry("one");
      YoVariableRegistry registryTwo = new YoVariableRegistry("two");
      YoVariableRegistry registryThree = new YoVariableRegistry("three");

      rootRegistry.addChild(registryOne);
      rootRegistry.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", rootRegistry);
      DoubleYoVariable variableTwo = new DoubleYoVariable("variableTwo", rootRegistry);
      BooleanYoVariable variableThree = new BooleanYoVariable("variableThree", rootRegistry);

      DoubleYoVariable variableFour = new DoubleYoVariable("variableFour", registryOne);
      IntYoVariable variableFive = new IntYoVariable("variableFive", registryOne);
      BooleanYoVariable variableSix = new BooleanYoVariable("variableSix", registryOne);

      DoubleYoVariable variableSeven = new DoubleYoVariable("variableSeven", registryTwo);
      DoubleYoVariable variableEight = new DoubleYoVariable("variableEight", registryTwo);
      BooleanYoVariable variableNine = new BooleanYoVariable("variableNine", registryTwo);

      DoubleYoVariable variableTen = new DoubleYoVariable("variableTen", registryThree);
      DoubleYoVariable variableEleven = new DoubleYoVariable("variableEleven", registryThree);
      BooleanYoVariable variableTwelve = new BooleanYoVariable("variableTwelve", registryThree);

      ArrayList<AbstractYoVariable> oddVariables = new ArrayList<AbstractYoVariable>();
      oddVariables.add(variableOne);
      oddVariables.add(variableThree);
      oddVariables.add(variableFive);
      oddVariables.add(variableSeven);
      oddVariables.add(variableNine);
      oddVariables.add(variableEleven);

      ArrayList<AbstractYoVariable> evenVariables = new ArrayList<AbstractYoVariable>();
      evenVariables.add(variableTwo);
      evenVariables.add(variableFour);
      evenVariables.add(variableSix);
      evenVariables.add(variableEight);
      evenVariables.add(variableTen);
      evenVariables.add(variableTwelve);

      if (readOddWriteEven)
      {
         whiteBoard.setVariablesToWrite(evenVariables);
         whiteBoard.setVariablesToRead(oddVariables);
      }
      else
      {
         whiteBoard.setVariablesToWrite(oddVariables);
         whiteBoard.setVariablesToRead(evenVariables);
      }
   }

}
