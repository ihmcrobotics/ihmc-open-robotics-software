package com.yobotics.simulationconstructionset.whiteBoard;


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import com.yobotics.simulationconstructionset.AbstractYoVariable;
import com.yobotics.simulationconstructionset.AbstractYoVariable.Type;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.NameSpace;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoWhiteBoardTest
{
   private static final boolean VERBOSE = false;

   @Test (expected = RuntimeException.class)
   public void testWriteNotConnected() throws IOException
   {
      YoWhiteBoard whiteBoard = new DoNothingWhiteBoard();
      whiteBoard.writeData();
   }

   @Test(expected = RuntimeException.class)
   public void testReadNotConnected() throws IOException
   {
      YoWhiteBoard whiteBoard = new DoNothingWhiteBoard();
      whiteBoard.readData();
   }

   protected void doATest(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard, int numberOfTests, int numberVariablesToReadOneWriteTwo, int numberVariablesToWriteOneReadTwo) throws IOException
   {
      createRandomRegistriesAndVariables(leftWhiteBoard, rightWhiteBoard, 20, numberVariablesToReadOneWriteTwo, numberVariablesToWriteOneReadTwo);

      leftWhiteBoard.connect();
      rightWhiteBoard.connect();

      YoWhiteBoardListenerForTest leftWhiteBoardListener = new YoWhiteBoardListenerForTest();
      YoWhiteBoardListenerForTest rightWhiteBoardListener = new YoWhiteBoardListenerForTest();

      leftWhiteBoard.attachYoWhiteBoardListener(leftWhiteBoardListener);
      rightWhiteBoard.attachYoWhiteBoardListener(rightWhiteBoardListener);

      long startTime = System.currentTimeMillis();

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

      for (int i = 0; i < numberOfTests; i++)
      {
         changeWrittenVariablesRandomly(leftWhiteBoard);
         changeWrittenVariablesRandomly(rightWhiteBoard);

         assertFalse(leftWhiteBoardListener.hasReceivedNewDataBeforeReset());
         assertFalse(rightWhiteBoardListener.hasReceivedNewDataBeforeReset());

         leftWhiteBoardListener.reset();
         rightWhiteBoardListener.reset();

         while (!leftWhiteBoard.isConnected())
         {
            if (VERBOSE) System.out.println("Waiting for left white board to connect.");
            try
            {
               Thread.sleep(1000);
            } 
            catch (InterruptedException e)
            {
            }
         }
         
         while (!rightWhiteBoard.isConnected())
         {
            if (VERBOSE) System.out.println("Waiting for right white board to connect.");
            try
            {
               Thread.sleep(1000);
            } 
            catch (InterruptedException e)
            {
            }
         }
         
         leftWhiteBoard.writeData();
         rightWhiteBoard.writeData();

         while (!leftWhiteBoardListener.isNewDataReady() ||!rightWhiteBoardListener.isNewDataReady())
         {
            Thread.yield();
         }

         leftWhiteBoard.readData();
         rightWhiteBoard.readData();

//       verifyWhiteBoardsHaveSameData(leftWhiteBoard, rightWhiteBoard);

         verifyYoVariablesHaveSameValues(leftVariablesToWrite, rightVariablesToRead);
         verifyYoVariablesHaveSameValues(leftVariablesToRead, rightVariablesToWrite);
         
         if (VERBOSE)
         {
            System.out.println("YoVariables had same data!");
         }
      }

      long endTime = System.currentTimeMillis();

      double duration = (endTime - startTime) * 0.001;

      if (VERBOSE)
         System.out.println("Ran " + numberOfTests + " tests in " + duration + " + seconds");
      double timePerTest = duration / ((double) numberOfTests);
      if (VERBOSE)
         System.out.println("Time per test = " + timePerTest);
      
      leftWhiteBoard.close();
      rightWhiteBoard.close();


   }

   private void verifyYoVariablesAreEqual(ArrayList<AbstractYoVariable> variablesOne, ArrayList<AbstractYoVariable> variablesTwo)
   {
      assertEquals(variablesOne.size(), variablesTwo.size());

      for (int i = 0; i < variablesOne.size(); i++)
      {
         verifyYoVariablesAreEqual(variablesOne.get(i), variablesTwo.get(i));
      }

   }


   private void verifyYoVariablesHaveSameValues(ArrayList<AbstractYoVariable> variablesOne, ArrayList<AbstractYoVariable> variablesTwo)
   {
      assertEquals(variablesOne.size(), variablesTwo.size());

      for (int i = 0; i < variablesOne.size(); i++)
      {
         verifyYoVariablesHaveSameValues(variablesOne.get(i), variablesTwo.get(i));
      }

   }

   private void verifyYoVariablesAreEqual(AbstractYoVariable variableOne, AbstractYoVariable variableTwo)
   {
      assertTrue(variableOne.getYoVariableType() == variableTwo.getYoVariableType());
      assertTrue(variableOne.getFullNameWithNameSpace().equals(variableTwo.getFullNameWithNameSpace()));

      assertEquals(variableOne.getValueAsDouble(), variableTwo.getValueAsDouble(), 1e-7);
   }

   private void verifyYoVariablesHaveSameValues(AbstractYoVariable variableOne, AbstractYoVariable variableTwo)
   {
      assertEquals(variableOne.getValueAsDouble(), variableTwo.getValueAsDouble(), 1e-7);
   }

   private final class DoNothingWhiteBoard extends YoWhiteBoard
   {
      public void whiteBoardSpecificWriteData(double[] doubleVariablesToWriteBuffer, int[] intVariablesToWriteBuffer, boolean[] booleanVariablesToWriteBuffer,
              int[] enumVariablesToWriteBuffer)
              throws IOException
      {
      }

      public void whiteBoardSpecificConnect() throws IOException
      {
      }

      public boolean isConnected()
      {
         return false;
      }

      public void whiteBoardSpecificClose() throws IOException
      {         
      }
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


   private void createVariableCopyFromReadToWrite(YoWhiteBoard boardToCopyFrom, YoWhiteBoard boardToCopyTo)
   {
      ArrayList<AbstractYoVariable> allVariableToRead = new ArrayList<AbstractYoVariable>();
      ArrayList<AbstractYoVariable> allVariableToWrite = new ArrayList<AbstractYoVariable>();
      boardToCopyFrom.getAllVariablesToRead(allVariableToRead);
      boardToCopyFrom.getAllVariablesToWrite(allVariableToWrite);


      ArrayList<AbstractYoVariable> copyVariablesToWrite = new ArrayList<AbstractYoVariable>();
      ArrayList<AbstractYoVariable> copyVariablesToRead = new ArrayList<AbstractYoVariable>();


      createVariableCopies(allVariableToRead, copyVariablesToWrite);
      createVariableCopies(allVariableToWrite, copyVariablesToRead);


      boardToCopyTo.setVariablesToWrite(copyVariablesToWrite);
      boardToCopyTo.setVariablesToRead(copyVariablesToRead);
   }


   private void createVariableCopies(ArrayList<AbstractYoVariable> variablesToCopy, ArrayList<AbstractYoVariable> variablesToCopyTo)
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("root");

      for (AbstractYoVariable variable : variablesToCopy)
      {
         String name = variable.getName();
         NameSpace nameSpace = variable.getNameSpace();

         Type yoVariableType = variable.getYoVariableType();
         YoVariableRegistry registry = rootRegistry.getOrCreateAndAddRegistry(nameSpace);

         switch (yoVariableType)
         {
            case DOUBLE :
            {
               variablesToCopyTo.add(new DoubleYoVariable(name, registry));

               break;
            }

            case INT :
            {
               variablesToCopyTo.add(new IntYoVariable(name, registry));

               break;
            }

            case BOOLEAN :
            {
               variablesToCopyTo.add(new BooleanYoVariable(name, registry));

               break;
            }

            case ENUM :
            {
               throw new RuntimeException("Help Twan!");

//             EnumYoVariable enumYoVariable = (EnumYoVariable) variable;
//             
//             variablesToCopyTo.add(new EnumYoVariable(name, registry));
//             break;
            }

            default :
            {
               throw new RuntimeException("Should not get here!");
            }
         }
      }

   }

   private void createRandomRegistriesAndVariables(YoWhiteBoard whiteBoardOne, YoWhiteBoard whiteBoardTwo, int numberOfRegistries,
           int numberVariablesToReadOneWriteTwo, int numberVariablesToWriteOneReadTwo)
   {
      createRandomRegistriesAndVariables(whiteBoardOne, numberOfRegistries, numberVariablesToReadOneWriteTwo, numberVariablesToWriteOneReadTwo);
      createVariableCopyFromReadToWrite(whiteBoardOne, whiteBoardTwo);
   }

   private void createRandomRegistriesAndVariables(YoWhiteBoard whiteBoard, int numberOfRegistries, int numberVariablesToRead, int numberVariablesToWrite)
   {
      Random random = new Random(1776L);

      ArrayList<YoVariableRegistry> registryList = generateRandomRegistries(random, numberOfRegistries);

      whiteBoard.setVariablesToRead(generateRandomVariables(random, "readVariable", numberVariablesToRead, registryList));
      whiteBoard.setVariablesToWrite(generateRandomVariables(random, "writeVariable", numberVariablesToRead, registryList));
   }

   private ArrayList<AbstractYoVariable> generateRandomVariables(Random random, String namePrefix, int numberOfVariables,
           ArrayList<YoVariableRegistry> registryList)
   {
      ArrayList<AbstractYoVariable> variables = new ArrayList<AbstractYoVariable>();

      for (int i = 0; i < numberOfVariables; i++)
      {
         int registryIndex = random.nextInt(registryList.size());
         YoVariableRegistry registry = registryList.get(registryIndex);

         int variableType = random.nextInt(3);

         String name = namePrefix + i;

         switch (variableType)
         {
            case 0 :
            {
               variables.add(new DoubleYoVariable(name, registry));

               break;
            }

            case 1 :
            {
               variables.add(new IntYoVariable(name, registry));

               break;
            }

            case 2 :
            {
               variables.add(new BooleanYoVariable(name, registry));

               break;
            }

            default :
            {
               throw new RuntimeException("Shouldn't get here!");
            }
         }
      }

      return variables;
   }


   private ArrayList<YoVariableRegistry> generateRandomRegistries(Random random, int numberOfRegistries)
   {
      ArrayList<YoVariableRegistry> ret = new ArrayList<YoVariableRegistry>();

      YoVariableRegistry rootRegistry = new YoVariableRegistry("root");
      ret.add(rootRegistry);

      for (int i = 0; i < numberOfRegistries; i++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("registry" + i);

         int registryIndex = random.nextInt(ret.size());
         YoVariableRegistry parentRegistry = ret.get(registryIndex);

         parentRegistry.addChild(registry);
         ret.add(registry);
      }

      return ret;
   }


}
