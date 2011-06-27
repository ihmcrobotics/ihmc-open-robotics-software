package com.yobotics.simulationconstructionset;


import static org.junit.Assert.*;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import com.yobotics.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;

public class DataFileWriterTest
{
   @BeforeClass
   public static void setUpBeforeClass() throws Exception
   {
   }

   @AfterClass
   public static void tearDownAfterClass() throws Exception
   {
   }

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }


   @Test
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
      IntYoVariable variableSeven = new IntYoVariable("variableSeven", registryThree);

      dataBuffer.addVariable(variableOne);
      dataBuffer.addVariable(variableTwo);
      dataBuffer.addVariable(variableThree);
      dataBuffer.addVariable(variableFour);
      dataBuffer.addVariable(variableFive);
      dataBuffer.addVariable(variableSix);
      dataBuffer.addVariable(variableSeven);
      
      for (int i=0; i<numDataPoints; i++)
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

      ArrayList<AbstractYoVariable> allVariables = rootRegistry.getAllVariablesIncludingDescendants();

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
   
   private void testDataWriteReadIsTheSame(DataBuffer dataBuffer, ArrayList<AbstractYoVariable> allVariables, 
         boolean binary, boolean compress, boolean spreadsheetFormatted, Robot robot) throws IOException
   {
      String filename = "testFile.data";
      if (spreadsheetFormatted) filename = filename + ".csv";
      if (compress) filename = filename + ".gz";
      
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

      VarList newVars = new VarList("newVars");

      dataFileReader.readData(newVars, readBackRegistry, readBackBuffer);
      
      
      boolean dataIsEqual = readBackBuffer.checkIfDataIsEqual(dataBuffer, 1e-7);
      assertTrue(dataIsEqual);
   }
}
