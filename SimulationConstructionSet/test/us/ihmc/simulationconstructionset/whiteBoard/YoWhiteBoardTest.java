package us.ihmc.simulationconstructionset.whiteBoard;


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableType;


public class YoWhiteBoardTest
{
   private static final boolean VERBOSE = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testWriteNotConnected() throws IOException
   {
      YoWhiteBoard whiteBoard = new DoNothingWhiteBoard();
      whiteBoard.writeData();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000,expected = RuntimeException.class)
   public void testReadNotConnected() throws IOException
   {
      YoWhiteBoard whiteBoard = new DoNothingWhiteBoard();
      whiteBoard.readData();
   }

   protected void doASynchronizedWriteThenReadTest(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard, int numberOfTests,
           int numberVariablesToReadOneWriteTwo, int numberVariablesToWriteOneReadTwo)
           throws IOException
   {
      createRandomRegistriesAndVariables(leftWhiteBoard, rightWhiteBoard, 20, numberVariablesToReadOneWriteTwo, numberVariablesToWriteOneReadTwo);

      leftWhiteBoard.connect();
      rightWhiteBoard.connect();

      YoWhiteBoardListenerForTest leftWhiteBoardListener = new YoWhiteBoardListenerForTest();
      YoWhiteBoardListenerForTest rightWhiteBoardListener = new YoWhiteBoardListenerForTest();

      leftWhiteBoard.attachYoWhiteBoardListener(leftWhiteBoardListener);
      rightWhiteBoard.attachYoWhiteBoardListener(rightWhiteBoardListener);

      long startTime = System.currentTimeMillis();

      ArrayList<YoVariable<?>> leftVariablesToWrite = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> leftVariablesToRead = new ArrayList<YoVariable<?>>();

      ArrayList<YoVariable<?>> rightVariablesToWrite = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> rightVariablesToRead = new ArrayList<YoVariable<?>>();

      leftWhiteBoard.getAllVariablesToWrite(leftVariablesToWrite);
      leftWhiteBoard.getAllVariablesToRead(leftVariablesToRead);

      rightWhiteBoard.getAllVariablesToWrite(rightVariablesToWrite);
      rightWhiteBoard.getAllVariablesToRead(rightVariablesToRead);

      verifyYoVariablesAreEqual(leftVariablesToWrite, rightVariablesToRead);
      verifyYoVariablesAreEqual(leftVariablesToRead, rightVariablesToWrite);

      waitForWhiteBoardsToConnect(leftWhiteBoard, rightWhiteBoard);

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
            Thread.yield();
         }

         assertEquals(1, leftWhiteBoard.getNumberOfNewDataSinceLastRead());
         assertEquals(1, rightWhiteBoard.getNumberOfNewDataSinceLastRead());

         verifyThatWhiteBoardsHaveNewDataAvailable(leftWhiteBoard, rightWhiteBoard);

         leftWhiteBoard.readData();
         rightWhiteBoard.readData();

         verifyThatWhiteBoardsDoNotHaveNewDataAvailable(leftWhiteBoard, rightWhiteBoard);

         verifyYoVariablesHaveSameValues(leftVariablesToWrite, rightVariablesToRead);
         verifyYoVariablesHaveSameValues(leftVariablesToRead, rightVariablesToWrite);
      }

      long endTime = System.currentTimeMillis();

      double duration = (endTime - startTime) * 0.001;

      if (VERBOSE)
         System.out.println("Ran " + numberOfTests + " tests in " + duration + " + seconds");
      double timePerTest = duration / ((double) numberOfTests);
      if (VERBOSE)
         System.out.println("Time per test = " + timePerTest);

      leftWhiteBoard.closeYoWhiteBoard();
      rightWhiteBoard.closeYoWhiteBoard();
   }


   protected void doAnAsynchronousTest(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard, int numberOfTests, int numberVariablesToReadOneWriteTwo,
           int numberVariablesToWriteOneReadTwo)
           throws IOException
   {
      createRandomRegistriesAndVariables(leftWhiteBoard, rightWhiteBoard, 20, numberVariablesToReadOneWriteTwo, numberVariablesToWriteOneReadTwo);

      leftWhiteBoard.connect();
      rightWhiteBoard.connect();

      long startTime = System.currentTimeMillis();

      ArrayList<YoVariable<?>> leftVariablesToWrite = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> leftVariablesToRead = new ArrayList<YoVariable<?>>();

      ArrayList<YoVariable<?>> rightVariablesToWrite = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> rightVariablesToRead = new ArrayList<YoVariable<?>>();

      leftWhiteBoard.getAllVariablesToWrite(leftVariablesToWrite);
      leftWhiteBoard.getAllVariablesToRead(leftVariablesToRead);

      rightWhiteBoard.getAllVariablesToWrite(rightVariablesToWrite);
      rightWhiteBoard.getAllVariablesToRead(rightVariablesToRead);

      verifyYoVariablesAreEqual(leftVariablesToWrite, rightVariablesToRead);
      verifyYoVariablesAreEqual(leftVariablesToRead, rightVariablesToWrite);

      waitForWhiteBoardsToConnect(leftWhiteBoard, rightWhiteBoard);

      Random random = new Random(1234);

      for (int i = 0; i < numberOfTests; i++)
      {
         int numberOfLeftWrites = 1 + random.nextInt(19);
         for (int j = 0; j < numberOfLeftWrites; j++)
         {
            changeWrittenVariablesRandomly(leftWhiteBoard);
            leftWhiteBoard.writeData();
         }

         int numberOfRightWrites = 1 + random.nextInt(19);
         for (int j = 0; j < numberOfRightWrites; j++)
         {
            changeWrittenVariablesRandomly(rightWhiteBoard);
            rightWhiteBoard.writeData();
         }

         while (rightWhiteBoard.getNumberOfNewDataSinceLastRead() < numberOfLeftWrites)
         {
            Thread.yield();
         }


         while (leftWhiteBoard.getNumberOfNewDataSinceLastRead() < numberOfRightWrites)
         {
            Thread.yield();
         }

         sleep(2);
         assertEquals(numberOfLeftWrites, rightWhiteBoard.getNumberOfNewDataSinceLastRead());
         assertEquals(numberOfRightWrites, leftWhiteBoard.getNumberOfNewDataSinceLastRead());

         verifyThatWhiteBoardsHaveNewDataAvailable(leftWhiteBoard, rightWhiteBoard);

         leftWhiteBoard.readData();
         rightWhiteBoard.readData();

         verifyThatWhiteBoardsDoNotHaveNewDataAvailable(leftWhiteBoard, rightWhiteBoard);

         verifyYoVariablesHaveSameValues(leftVariablesToWrite, rightVariablesToRead);
         verifyYoVariablesHaveSameValues(leftVariablesToRead, rightVariablesToWrite);
      }

      long endTime = System.currentTimeMillis();

      double duration = (endTime - startTime) * 0.001;

      if (VERBOSE)
         System.out.println("Ran " + numberOfTests + " tests in " + duration + " + seconds");
      double timePerTest = duration / ((double) numberOfTests);
      if (VERBOSE)
         System.out.println("Time per test = " + timePerTest);

      leftWhiteBoard.closeYoWhiteBoard();
      rightWhiteBoard.closeYoWhiteBoard();
   }

   private void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      }
      catch (InterruptedException e)
      {
      }

   }

   private void verifyThatWhiteBoardsDoNotHaveNewDataAvailable(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard)
   {
      assertFalse(leftWhiteBoard.isNewDataAvailable());
      assertFalse(rightWhiteBoard.isNewDataAvailable());

      assertEquals(0, leftWhiteBoard.getNumberOfNewDataSinceLastRead());
      assertEquals(0, rightWhiteBoard.getNumberOfNewDataSinceLastRead());
   }

   private void verifyThatWhiteBoardsHaveNewDataAvailable(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard)
   {
      assertTrue(leftWhiteBoard.isNewDataAvailable());
      assertTrue(rightWhiteBoard.isNewDataAvailable());
   }

   private void waitForWhiteBoardsToConnect(YoWhiteBoard leftWhiteBoard, YoWhiteBoard rightWhiteBoard)
   {
      while (!leftWhiteBoard.isConnected())
      {
         if (VERBOSE)
            System.out.println("Waiting for left white board to connect.");

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
         if (VERBOSE)
            System.out.println("Waiting for right white board to connect.");

         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }
      }
   }

   private void verifyYoVariablesAreEqual(ArrayList<YoVariable<?>> variablesOne, ArrayList<YoVariable<?>> variablesTwo)
   {
      assertEquals(variablesOne.size(), variablesTwo.size());

      for (int i = 0; i < variablesOne.size(); i++)
      {
         verifyYoVariablesAreEqual(variablesOne.get(i), variablesTwo.get(i));
      }

   }


   private void verifyYoVariablesHaveSameValues(ArrayList<YoVariable<?>> variablesOne, ArrayList<YoVariable<?>> variablesTwo)
   {
      assertEquals(variablesOne.size(), variablesTwo.size());

      for (int i = 0; i < variablesOne.size(); i++)
      {
         verifyYoVariablesHaveSameValues(variablesOne.get(i), variablesTwo.get(i));
      }

   }

   private void verifyYoVariablesAreEqual(YoVariable<?> variableOne, YoVariable<?> variableTwo)
   {
      assertTrue(variableOne.getYoVariableType() == variableTwo.getYoVariableType());
      assertTrue(variableOne.getFullNameWithNameSpace().equals(variableTwo.getFullNameWithNameSpace()));

      assertEquals(variableOne.getValueAsDouble(), variableTwo.getValueAsDouble(), 1e-7);
   }

   private void verifyYoVariablesHaveSameValues(YoVariable<?> variableOne, YoVariable<?> variableTwo)
   {
      assertEquals(variableOne.getValueAsDouble(), variableTwo.getValueAsDouble(), 1e-7);
   }

   private final class DoNothingWhiteBoard extends YoWhiteBoard
   {
      public DoNothingWhiteBoard()
      {
         super("DoNothing", new YoVariableRegistry("DoNothingWhiteBoard"));
      }
      
      @Override
      public void whiteBoardSpecificWriteData(double[] doubleVariablesToWriteBuffer, int[] intVariablesToWriteBuffer, boolean[] booleanVariablesToWriteBuffer,
              int[] enumVariablesToWriteBuffer, int writeIndex)
              throws IOException
      {
      }

      @Override
      public void whiteBoardSpecificConnect() throws IOException
      {
      }

      @Override
      public void closeYoWhiteBoard() throws IOException
      {
      }
   }

   private class YoWhiteBoardListenerForTest implements YoWhiteBoardListener
   {
      private boolean newDataReady = false;
      private boolean hasReceivedNewDataBeforeReset = false;

      @Override
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
      ArrayList<YoVariable<?>> variablesToWrite = new ArrayList<YoVariable<?>>();
      leftWhiteBoard.getAllVariablesToWrite(variablesToWrite);

      for (YoVariable<?> variable : variablesToWrite)
      {
         variable.setValueFromDouble(2.0 * Math.random());
      }
   }


   private void createVariableCopyFromReadToWrite(YoWhiteBoard boardToCopyFrom, YoWhiteBoard boardToCopyTo)
   {
      ArrayList<YoVariable<?>> allVariableToRead = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> allVariableToWrite = new ArrayList<YoVariable<?>>();
      boardToCopyFrom.getAllVariablesToRead(allVariableToRead);
      boardToCopyFrom.getAllVariablesToWrite(allVariableToWrite);


      ArrayList<YoVariable<?>> copyVariablesToWrite = new ArrayList<YoVariable<?>>();
      ArrayList<YoVariable<?>> copyVariablesToRead = new ArrayList<YoVariable<?>>();


      createVariableCopies(allVariableToRead, copyVariablesToWrite);
      createVariableCopies(allVariableToWrite, copyVariablesToRead);


      boardToCopyTo.setVariablesToWrite(copyVariablesToWrite);
      boardToCopyTo.setVariablesToRead(copyVariablesToRead);
   }


   private void createVariableCopies(ArrayList<YoVariable<?>> variablesToCopy, ArrayList<YoVariable<?>> variablesToCopyTo)
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("root");

      for (YoVariable<?> variable : variablesToCopy)
      {
         String name = variable.getName();
         NameSpace nameSpace = variable.getNameSpace();

         YoVariableType yoVariableType = variable.getYoVariableType();
         YoVariableRegistry registry = rootRegistry.getOrCreateAndAddRegistry(nameSpace);

         switch (yoVariableType)
         {
            case DOUBLE :
            {
               variablesToCopyTo.add(new DoubleYoVariable(name, registry));

               break;
            }

            case INTEGER :
            {
               variablesToCopyTo.add(new IntegerYoVariable(name, registry));

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
      whiteBoard.setVariablesToWrite(generateRandomVariables(random, "writeVariable", numberVariablesToWrite, registryList));
   }

   private ArrayList<YoVariable<?>> generateRandomVariables(Random random, String namePrefix, int numberOfVariables, ArrayList<YoVariableRegistry> registryList)
   {
      ArrayList<YoVariable<?>> variables = new ArrayList<YoVariable<?>>();

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
               variables.add(new IntegerYoVariable(name, registry));

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
