package us.ihmc.simulationconstructionset;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Random;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.ExpectedException;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;

public class DataFileWriterTest
{
   private static final String TEST_DIRECTORY = "testResources/us/ihmc/simulationconstructionset/dataFileWriterTest/";
   
   @Rule
   public ExpectedException expectedException = ExpectedException.none();

	@ContinuousIntegrationTest(estimatedDuration = 0.7)
	@Test(timeout=300000)
   public void testDataFileWriterAndReader() throws IOException, RepeatDataBufferEntryException
   {
      int numDataPoints = 10000;

      DataBuffer dataBuffer = new DataBuffer(numDataPoints);

      YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRegistry");
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");
      YoVariableRegistry registryThree = new YoVariableRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      rootRegistry.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      DoubleYoVariable variableOne = new DoubleYoVariable("variableOne", rootRegistry);
      DoubleYoVariable variableTwo = new DoubleYoVariable("variableTwo", rootRegistry);
      DoubleYoVariable variableThree = new DoubleYoVariable("variableThree", rootRegistry);
      DoubleYoVariable variableFour = new DoubleYoVariable("variableFour", registryOne);
      DoubleYoVariable variableFive = new DoubleYoVariable("variableFive", registryTwo);
      BooleanYoVariable variableSix = new BooleanYoVariable("variableSix", rootRegistry);
      IntegerYoVariable variableSeven = new IntegerYoVariable("variableSeven", registryThree);

      dataBuffer.addVariable(variableOne);
      dataBuffer.addVariable(variableTwo);
      dataBuffer.addVariable(variableThree);
      dataBuffer.addVariable(variableFour);
      dataBuffer.addVariable(variableFive);
      dataBuffer.addVariable(variableSix);
      dataBuffer.addVariable(variableSeven);

      for (int i = 0; i < numDataPoints; i++)
      {
         variableOne.set(Math.random());
         variableTwo.set(Math.random());
         variableThree.set((int) (Math.random() * 100.0));
         variableFour.set((int) (Math.random() * 100.0));
         variableFive.set(Math.random());
         variableSix.set(Math.random() > 0.5);
         variableSeven.set((int) (Math.random() * 1000.0));

         dataBuffer.tickAndUpdate();
      }

      Robot robot = new Robot("testRobot");

      ArrayList<YoVariable<?>> allVariables = rootRegistry.getAllVariablesIncludingDescendants();

      boolean binary = false;
      boolean compress = false;
      boolean spreadsheetFormatted = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      spreadsheetFormatted = false;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = true;
      compress = false;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = false;
      compress = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = true;
      compress = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

   }

   private void testDataWriteReadIsTheSame(DataBuffer dataBuffer, ArrayList<YoVariable<?>> allVariables, boolean binary, boolean compress,
           boolean spreadsheetFormatted, Robot robot)
           throws IOException
   {
      String filename = TEST_DIRECTORY + "testFile.data";
      if (spreadsheetFormatted)
         filename = filename + ".csv";
      if (compress)
         filename = filename + ".gz";

      File testFile = new File(filename);

      String model = "testModel";
      double recordDT = 0.001;

      DataFileWriter dataFileWriter = new DataFileWriter(testFile);
      if (spreadsheetFormatted)
      {
         dataFileWriter.writeSpreadsheetFormattedData(dataBuffer, allVariables);

      }
      else
      {
         dataFileWriter.writeData(model, recordDT, dataBuffer, allVariables, binary, compress, robot);
      }

      DataFileReader dataFileReader = new DataFileReader(testFile);
      DataBuffer readBackBuffer = new DataBuffer(dataBuffer.getBufferSize());
      YoVariableRegistry readBackRegistry = new YoVariableRegistry("rootRegistry");

      YoVariableList newVars = new YoVariableList("newVars");

      dataFileReader.readData(newVars, readBackRegistry, readBackBuffer);

      boolean dataIsEqual = readBackBuffer.checkIfDataIsEqual(dataBuffer, 1e-7);
      assertTrue(dataIsEqual);
   }

   @SuppressWarnings("deprecation")

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFileReadAndWriteWithDataOutputStreamAndDataInputStream() throws IOException, FileNotFoundException, NullPointerException
   {
      Random rng = new Random();
      String testString = "This string tests readLine";
      double testDouble = rng.nextDouble();
      int testInteger = rng.nextInt();
      File testFile = new File(TEST_DIRECTORY + "shortReadWriteTestFile.txt");

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeDouble(testDouble);
      outputStream.writeBytes(testString + "\n");
      outputStream.writeInt(testInteger);
      outputStream.close();

      DataInputStream inputStream = new DataInputStream(new FileInputStream(testFile));
      double doubleReadBack = inputStream.readDouble();
      String lineReadBack = inputStream.readLine();
      int integerReadBack = inputStream.readInt();

      assertTrue(testDouble == doubleReadBack);
      assertTrue(testString.equals(lineReadBack));
      assertTrue(testInteger == integerReadBack);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFileReadAndWriteWithDataOutputStreamAndBufferedReader() throws FileNotFoundException, IOException
   {
      expectedException.expect(EOFException.class);

      Random rng = new Random();
      String testString = "This string tests readLine";
      double testDouble = rng.nextDouble();
      int testInteger = rng.nextInt();
      File testFile = new File(TEST_DIRECTORY + "shortReadWriteTestFile.txt");

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeDouble(testDouble);
      outputStream.writeBytes(testString + "\n");
      outputStream.writeInt(testInteger);
      outputStream.close();

      DataInputStream inputStream = new DataInputStream(new FileInputStream(testFile));
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream, "US-ASCII"));
      double doubleReadBack = inputStream.readDouble();
      String lineReadBack = bufferedReader.readLine();
      int integerReadBack = inputStream.readInt();
      integerReadBack = -testInteger;

      inputStream.close();
      bufferedReader.close();

      // System.out.println(lineReadBack);
      // System.out.println(testString);

      assertTrue(testDouble == doubleReadBack);
      assertTrue(testString.equals(lineReadBack));
      assertTrue(integerReadBack == -testInteger);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFileReadAndWriteBackWithDataOutputStreamAndDeferredBufferedReaderCreation() throws IOException
   {
      Random rng = new Random();
      String testString = "This string tests readLine";
      double testDouble = rng.nextDouble();
      int testInteger = rng.nextInt();
      File testFile = new File(TEST_DIRECTORY + "shortReadWriteTestFile.txt");

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeDouble(testDouble);
      outputStream.writeInt(testInteger);
      outputStream.writeBytes(testString + "\n");
      outputStream.close();

      DataInputStream inputStream = new DataInputStream(new FileInputStream(testFile));
      double doubleReadBack = inputStream.readDouble();
      int integerReadBack = inputStream.readInt();

      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream, "US-ASCII"));
      String lineReadBack = bufferedReader.readLine();

      inputStream.close();
      bufferedReader.close();

      System.out.println(lineReadBack);
      System.out.println(testString);

      assertTrue(testDouble == doubleReadBack);
      assertTrue(testString.equals(lineReadBack));
      assertTrue(testInteger == integerReadBack);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFileReadAndWriteBackWithDataOutputStreamAndBufferedReaderStringsOnly() throws IOException
   {
      String string1 = "This is the first string";
      String string2 = "This is the second string";
      String string3 = "This is the third string";
      File testFile = new File(TEST_DIRECTORY + "shortReadWriteTestFile.txt");

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeBytes(string1 + "\n");
      outputStream.writeBytes(string2 + "\n");
      outputStream.writeBytes(string3 + "\n");
      outputStream.close();

      BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(testFile), "US-ASCII"));

      String readBack1 = reader.readLine();
      String readBack2 = reader.readLine();
      String readBack3 = reader.readLine();

      reader.close();

      // System.out.println("String 1: " + string1);
      // System.out.println("String 2: " + string2);
      // System.out.println("String 3: " + string3);
      //
      // System.out.println("Readback 1: " + readBack1);
      // System.out.println("Readback 2: " + readBack2);
      // System.out.println("Readback 3: " + readBack3);

      assertTrue(string1.equals(readBack1));
      assertTrue(string2.equals(readBack2));
      assertTrue(string3.equals(readBack3));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testWritingAndReadingALongStateFile() throws IOException
   {
      File fileOne = new File(TEST_DIRECTORY + "fileOne.state");

      if (fileOne.exists())
         fileOne.delete();

      long seed = 1776L;
      int numberOfVariables = 2000;    // 12000 for when testing long files for efficiency;
      Random random = new Random(seed);
      ArrayList<YoVariable<?>> variables = createALargeNumberOfVariables(random, numberOfVariables);
      YoVariableList originalVarList = new YoVariableList("originalVarList");
      originalVarList.addVariables(variables);

      writeALongStateFile(fileOne, variables);

      DataFileReader dataFileReader = new DataFileReader(fileOne);

      YoVariableList newVarList = new YoVariableList("newVarList");
      boolean createMissingVariables = true;
      boolean printErrorForMissingVariables = false;
      YoVariableRegistry registry = new YoVariableRegistry("root");

      dataFileReader.readState(newVarList, createMissingVariables, printErrorForMissingVariables, registry);

      assertEquals(originalVarList.size(), newVarList.size());

      for (int i = 0; i < originalVarList.size(); i++)
      {
         YoVariable<?> originalVariable = originalVarList.getVariable(i);
         YoVariable<?> newVariable = newVarList.getVariable(originalVariable.getName());

         assertFalse(originalVariable == newVariable);
         assertEquals(originalVariable.getValueAsDouble(), newVariable.getValueAsDouble(), 1e-7);

      }

      fileOne.delete();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testWritingAndReadingADataFileWithLotsOfVariables() throws IOException, RepeatDataBufferEntryException
   {
      File fileOne = new File(TEST_DIRECTORY + "fileOne.data.gz");

      if (fileOne.exists())
         fileOne.delete();

      long seed = 1776L;
      int numberOfVariables = 2000;    // 12000 for when testing long files for efficiency;
      Random random = new Random(seed);
      ArrayList<YoVariable<?>> variables = createALargeNumberOfVariables(random, numberOfVariables);
      YoVariableList originalVarList = new YoVariableList("originalVarList");
      originalVarList.addVariables(variables);

      int bufferSize = 50;
      DataBuffer dataBuffer = new DataBuffer(bufferSize);
      
      dataBuffer.addVariables(variables);
      
      for (int i=0; i<bufferSize/2; i++)
      {
         dataBuffer.setDataAtIndexToYoVariableValues();
         dataBuffer.tick(1);
      }
      
      
      dataBuffer.setInOutPointFullBuffer();
      
      Robot robot = new Robot("testWritingRobot");
      writeALongDataFile(fileOne, dataBuffer, variables, robot);

      System.out.println("Wrote File. Now reading it.");
      
      DataFileReader dataFileReader = new DataFileReader(fileOne);

      YoVariableList newVarList = new YoVariableList("newVarList");
      YoVariableRegistry registry = new YoVariableRegistry("rootRegistry");

      DataBuffer newDataBuffer = new DataBuffer();
      dataFileReader.readData(newVarList, registry, newDataBuffer);

      assertEquals(originalVarList.size(), newVarList.size());

      for (int i = 0; i < originalVarList.size(); i++)
      {
         YoVariable<?> originalVariable = originalVarList.getVariable(i);
         YoVariable<?> newVariable = newVarList.getVariable(originalVariable.getName());

         assertFalse(originalVariable == newVariable);
         assertEquals(originalVariable.getValueAsDouble(), newVariable.getValueAsDouble(), 1e-7);

      }

      fileOne.delete();
   }

   private void writeALongStateFile(File file, ArrayList<YoVariable<?>> variables)
   {
      DataFileWriter dataFileWriter = new DataFileWriter(file);

      boolean compress = false;
      double recordDT = 0.001;
      boolean binary = false;
      dataFileWriter.writeState("model", recordDT, variables, binary, compress);
   }
   
   private void writeALongDataFile(File file, DataBuffer dataBuffer, ArrayList<YoVariable<?>> variables, Robot robot)
   {
      DataFileWriter dataFileWriter = new DataFileWriter(file);

      boolean compress = true;
      double recordDT = 0.001;
      boolean binary = true;
      dataFileWriter.writeData("model", recordDT, dataBuffer, variables, binary, compress, robot);
   }


   private ArrayList<YoVariable<?>> createALargeNumberOfVariables(Random random, int numberOfVariables)
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRegistry");
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");
      YoVariableRegistry registryThree = new YoVariableRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      DoubleYoVariable t = new DoubleYoVariable("t", registryThree);
      DoubleYoVariable time = new DoubleYoVariable("time", registryThree);
      t.set(1.1);
      time.set(2.2);

      for (int i = 0; i < numberOfVariables; i++)
      {
         DoubleYoVariable variable = new DoubleYoVariable("variable" + i, registryThree);
         variable.set(Math.random());
      }

      return rootRegistry.getAllVariablesIncludingDescendants();
   }


}
