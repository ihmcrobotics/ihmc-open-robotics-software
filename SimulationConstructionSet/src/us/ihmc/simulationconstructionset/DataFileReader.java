package us.ihmc.simulationconstructionset;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;
import java.util.zip.GZIPInputStream;

import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

public class DataFileReader
{
   private static final boolean DEBUG = false;
   private boolean columnFormatted;

   // String filename;
   private File inFile;
   private URL inURL;

   private int nVars;
   private int nPoints;
   private double recordDT;
   private ArrayList<String> varNames;

   // private ArrayList variables;

   public DataFileReader(File file)    // String filename)
   {
      this.inFile = file;
      this.inURL = null;
   }

   public DataFileReader(URL url)    // String filename)
   {
      this.inURL = url;
      this.inFile = null;
   }


   // public ArrayList getVariables(){return this.variables;}

   public double getRecordDT()
   {
      return this.recordDT;
   }

   private boolean parseLine(String line)
   {
      boolean ret = false;
      StringTokenizer st = new StringTokenizer(line);

      try
      {
         String nT = st.nextToken();

//       System.out.println(".");
//         System.out.println(nT);

         if (nT.equals("$END_HEADER"))
         {
            ret = true;
         }
         else if (nT.equals("$NVAR"))
         {
            nVars = Integer.parseInt(st.nextToken());
            System.out.println("nVars equals: " + nVars);
         }
         else if (nT.equals("$DT"))
         {
            recordDT = Double.parseDouble(st.nextToken());
            System.out.println("recordDT equals: " + recordDT);
         }
         else if (nT.equals("$N"))
         {
            nPoints = Integer.parseInt(st.nextToken());
            System.out.println("nPoints equals: " + nPoints);
         }

         else if (nT.equals("$COLUMN"))
         {
            System.out.println("Column Formatted");
            columnFormatted = true;
         }

         else if (nT.equals("$ROW"))
         {
            System.out.println("Row Formatted");
            columnFormatted = false;
         }

         else if (nT.equals("$VAR"))
         {
            String vName = st.nextToken();

//          vName = vName.replace('.', '_');

            varNames.add(vName);
         }
      }

      catch (NoSuchElementException noSuchElementException)
      {
         System.err.println("No Such Element Exception!! " + noSuchElementException);

         throw new RuntimeException("No Such Element " + noSuchElementException);
      }

      return ret;
   }

   public int readData(YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer) throws IOException
   {
      return readData(newVars, rootRegistryToAddNewVariablesTo, dataBuffer, null);
   }

   public int readData(YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer, SimulationConstructionSet sim)
           throws IOException
   {
      if (inFile != null)
         return readDataFile(newVars, rootRegistryToAddNewVariablesTo, dataBuffer, sim);
      else if (inURL != null)
         return readDataURL(newVars, rootRegistryToAddNewVariablesTo, dataBuffer, sim);
      else
         return -1;
   }

   private int readDataFile(YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer, SimulationConstructionSet sim)
           throws IOException
   {
//    try
//    {
      YoDataInputStream dataStream;

      if (inFile.getName().endsWith(".gz"))
         dataStream = new YoDataInputStream(new BufferedInputStream(new GZIPInputStream(new BufferedInputStream(new FileInputStream(inFile)))));    // inStream);
      else
         dataStream = new YoDataInputStream(new BufferedInputStream(new FileInputStream(inFile)));    // inStream);

      if (inFile.getName().endsWith(".csv"))
      {
         return readASCIICommaSeparatedData(dataStream, newVars, dataBuffer, rootRegistryToAddNewVariablesTo);
      }

      return readData(dataStream, newVars, rootRegistryToAddNewVariablesTo, dataBuffer, sim);

//    }
//    catch (IOException e)
//    {
//      return -1;
//    }

   }

   private int readDataURL(YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer, SimulationConstructionSet sim)
           throws IOException
   {
//    try
//    {
      YoDataInputStream dataStream;

      if (inURL.getPath().endsWith(".gz"))
         dataStream = new YoDataInputStream(new BufferedInputStream(new GZIPInputStream(new BufferedInputStream(inURL.openStream()))));    // inStream);

      else
         dataStream = new YoDataInputStream(new BufferedInputStream(inURL.openStream()));    // inStream);

      if (inURL.getPath().endsWith(".csv"))
      {
         return readASCIICommaSeparatedData(dataStream, newVars, dataBuffer, rootRegistryToAddNewVariablesTo);
      }

      return readData(dataStream, newVars, rootRegistryToAddNewVariablesTo, dataBuffer, sim);

//    }
//    catch (IOException e)
//    {
//      return -1;
//    }

   }

   public int readData(YoDataInputStream dataStream, YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer,
                       SimulationConstructionSet sim)
           throws IOException
   {
      nPoints = -1;
      varNames = new ArrayList<String>();

      // variables = new ArrayList();

//    try
//      {
      String line = dataStream.readASCIILine();

      // System.out.println(line);
      if (!line.startsWith("$BEGIN_HEADER"))
      {
         return readASCIIData(dataStream, newVars, dataBuffer, line, rootRegistryToAddNewVariablesTo);
      }

      boolean doneParsing = false;
      boolean inRobotConfig = false;

      String robotConfig = "";

      while (((!doneParsing) && (line = dataStream.readASCIILine()) != null))
      {
         if (line.startsWith("$<RobotDefinition>"))
         {
            inRobotConfig = true;
            robotConfig += line.substring(1, line.length()) + "\n";
         }
         else if (line.startsWith("$</RobotDefinition>"))
         {
            robotConfig += line + "\n";
            inRobotConfig = false;
         }
         else if (inRobotConfig)
         {
            robotConfig += line + "\n";
         }
         else if (!inRobotConfig)
         {
            doneParsing = parseLine(line);
         }

         if (DEBUG)
            System.out.println("Parsed line " + line);
      }

      robotConfig = replaceAll(robotConfig, "\n$", "\n");

      RobotDefinitionFixedFrame compare = new RobotDefinitionFixedFrame();

      if (sim != null)
      {
         Robot robot = sim.getRobots()[0];
         compare.createRobotDefinitionFromRobot(robot);

         if (robotConfig.toString().equals(compare.toString()))
         {
            // System.out.println("The robots match.");
         }
         else
         {
            System.err.println("Warning: The robots do not match.");
         }
      }

      if (DEBUG)
         System.out.println("Done Parsing!");

//    if (nPoints > 0)
      {
         // dataBuffer.setMaxIndex(nPoints);
         // First Clear all the existing data in case we don't read one of the variables that currently exists.

         // vars.clearAll(nPoints);
         if (nPoints != -1)
         {
            dataBuffer.clearAll(nPoints);
         }

         else
         {
            dataBuffer.clearAll(1024);
         }

         if (columnFormatted)
            loadColumnFormattedData(dataStream, newVars, rootRegistryToAddNewVariablesTo, dataBuffer);
         else
            loadRowFormattedData(dataStream, newVars, rootRegistryToAddNewVariablesTo, dataBuffer);

         dataBuffer.setInPoint(0);
         dataBuffer.setOutPoint(nPoints - 1);
         dataBuffer.setIndex(0);
      }

      // vars.setMaxIndex(nPoints);

      dataStream.close();

      // inStream.close();

//    }
//    catch (IOException e)
//    {
////       e.printStackTrace();
//       return -1;
//    }


      // int lB3 = Integer.parseInt("42913ac7",16);
      // float foo3 = Float.intBitsToFloat(lB3);
      // String hexS3 = Integer.toHexString(lB3);

      // System.out.println(lB3);
      // System.out.println(hexS3);
      // System.out.println(foo3);

      return nPoints;
   }

   private String replaceAll(String orig, String regex, String rep)
   {
      int index = 0;
      while (index + regex.length() < orig.length())
      {
         if (orig.substring(index, index + regex.length()).equals(regex))
         {
            orig = replace(orig, index, index + regex.length(), rep);
            index += rep.length() - 1;
         }

         index++;
      }

      return orig;
   }

   private String replace(String fullString, int beginIndex, int endIndex, String replacement)
   {
      if ((beginIndex >= 0) && (beginIndex < fullString.length()) && (endIndex >= 0) && (endIndex < fullString.length()))
      {
         String prefix = fullString.substring(0, beginIndex);
         String suffix = fullString.substring(endIndex);

         return prefix + replacement + suffix;
      }
      else
      {
         return null;
      }
   }

   private void loadColumnFormattedData(YoDataInputStream dataStream, YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer)
           throws IOException
   {
      for (int i = 0; i < nVars; i++)
      {
         DataBufferEntry newEntry = getDataBufferEntry(varNames.get(i), dataBuffer, rootRegistryToAddNewVariablesTo, newVars);


//       System.out.println("varNames.get(i) = " + varNames.get(i) + ", newEntry = " + newEntry);


         double[] someData = newEntry.getData();    // new double[nPoints];

         for (int j = 0; j < nPoints; j++)
         {
            someData[j] = (dataStream.readFloat());

            // System.out.print(dataStream.readFloat() + "  ");
         }

         // System.out.println(varNames.get(i));
         // dataHolder.addData((String) varNames.get(i),someData);
         // newVariable.setData(someData,nPoints);
         newEntry.setData(someData, nPoints);

         // variables.add(newVariable);
         // System.out.println();
      }
   }

   private DataBufferEntry getDataBufferEntry(String varName, DataBuffer dataBuffer, YoVariableRegistry rootRegistryToAddNewVariablesTo, YoVariableList newVars)
           throws IOException
   {
      YoVariable<?> newVariable = dataBuffer.getVariable(varName);

      if (newVariable == null)
      {
         NameSpace nameSpace = NameSpace.createNameSpaceFromAFullVariableName(varName);
         String variableName = NameSpace.stripOffNameSpaceToGetVariableName(varName);

         YoVariableRegistry registry = rootRegistryToAddNewVariablesTo.getOrCreateAndAddRegistry(nameSpace);

         newVariable = new DoubleYoVariable(variableName, "Created Variable in DataFileReader", registry);
         newVars.addVariable(newVariable);
      }

      DataBufferEntry newEntry = dataBuffer.getEntry(newVariable);

      if (newEntry == null)
      {
         try
         {
            if (nPoints != -1)
            {
               newEntry = dataBuffer.addVariable(newVariable, nPoints);
            }
            else
            {
               newEntry = dataBuffer.addVariable(newVariable, dataBuffer.getBufferSize());
            }

//            newEntry = dataBuffer.getEntry(varName);
         }
         catch (RepeatDataBufferEntryException ex)
         {
            throw new IOException("Repeat Variable in file: " + newVariable.getName());
         }
      }

      return newEntry;
   }

   private void loadRowFormattedData(YoDataInputStream dataStream, YoVariableList newVars, YoVariableRegistry rootRegistryToAddNewVariablesTo, DataBuffer dataBuffer)
           throws IOException
   {
      if (DEBUG)
         System.err.println("Reading Row Formatted Data");

//    try
//    {
      DataBufferEntry[] entries = new DataBufferEntry[nVars];

      for (int i = 0; i < nVars; i++)
      {
//       System.err.println("i = " + i);


         entries[i] = getDataBufferEntry(varNames.get(i), dataBuffer, rootRegistryToAddNewVariablesTo, newVars);
      }

      int j = 0;
      try
      {
         int numberOfPointsToIterateOver;
         if (nPoints != -1)
            numberOfPointsToIterateOver = nPoints;
         else
            numberOfPointsToIterateOver = Integer.MAX_VALUE;

         for (j = 0; j < numberOfPointsToIterateOver; j++)
         {
//          if (DEBUG) System.err.println("j = " + j);

            if (dataBuffer.getBufferSize() <= j)
            {
               int newBufferSize = Math.min(j, Integer.MAX_VALUE / 2);
               newBufferSize *= 2;
               dataBuffer.changeBufferSize(newBufferSize);
            }

            for (int i = 0; i < nVars; i++)
            {
               double someData = (dataStream.readFloat());

               // System.out.print(someData);
               DataBufferEntry entry = entries[i];
               entry.setData(someData, j);
            }

            // System.out.println();

         }
      }
      catch (IOException iOException)
      {
         // +++JEP080725: This can happen for logged data files, where we put the nPoints to a large number since we don't know what it'll be!!
         if (DEBUG)
            System.out.println("Caught Exception while reading data: " + iOException + ". Didn't find nPoints in the data! Only got to " + j + " out of "
                               + nPoints);

         if (nPoints == -1)
            nPoints = j;
      }

//    for (int i=0; i<nVars; i++)
//    {
//       entries[i].reCalcMinMax();
//    }

      // variables.add(newVariable);
      // System.out.println();
//    }
//    catch(Exception e)
//    {
//       e.printStackTrace();
//    }

   }

   private int readASCIIData(YoDataInputStream dataStream, YoVariableList newVars, DataBuffer dataBuffer, String line,
                             YoVariableRegistry rootRegistryToAddNewVariablesTo)
           throws IOException
   {
      nPoints = -1;
      varNames = new ArrayList<String>();

//    try
//      {
      while (!(line == null))
      {
         // System.out.println(line);
         StringTokenizer token = new StringTokenizer(line);

         // First get the varname:
         String varName = token.nextToken();

         // System.out.print(varName + ": ");

         // Next token must be equals:
         String equals = token.nextToken();
         if (!equals.equals("="))
            return -1;

         // Next token must eith be the value of DT or start with [

         if (varName.equals("DT"))
         {
            String valString = token.nextToken();

            // Strip off the semicolon at the end:
            if (!valString.endsWith(";"))
               return -1;

            valString = valString.substring(0, valString.indexOf(";"));
            recordDT = Double.parseDouble(valString);
         }

         else
         {
            if (nPoints == -1)
            {
               nPoints = token.countTokens() - 1;
               dataBuffer.clearAll(nPoints);

               // System.out.println("nPoints = " + nPoints);
            }


            DataBufferEntry newEntry = getDataBufferEntry(varName, dataBuffer, rootRegistryToAddNewVariablesTo, newVars);



            double[] someData = newEntry.getData();

            String valString = token.nextToken();
            if (!valString.startsWith("["))
               return -1;

            int point = 0;
            valString = valString.substring(1);

            while (!valString.startsWith("];"))
            {
               double value = Double.parseDouble(valString);

               // System.out.print(value + " ");
               someData[point] = (value);
               point++;

               if (!token.hasMoreTokens())
                  return -1;
               valString = token.nextToken();
            }

            if (nPoints != point)
               return -1;
            newEntry.setData(someData, nPoints);

            // System.out.println();
         }

         line = dataStream.readASCIILine();
      }

      dataStream.close();
      System.out.println("nPoints = " + nPoints);
      System.out.println("recordDT equals: " + recordDT);
      dataBuffer.setInPoint(0);
      dataBuffer.setOutPoint(nPoints - 1);
      dataBuffer.setIndex(0);

//    }
//    catch (IOException e){return -1;}
      return nPoints;
   }

   private int readASCIICommaSeparatedData(YoDataInputStream dataStream, YoVariableList newVars, DataBuffer dataBuffer,
           YoVariableRegistry rootRegistryToAddNewVariablesTo)
           throws IOException
   {
      nPoints = -1;
      varNames = new ArrayList<String>();

      ArrayList<double[]> dataArrays = new ArrayList<double[]>();

//    try
//    {
      String line = dataStream.readASCIILine();    // Read the variable names:
      StringTokenizer token = new StringTokenizer(line, ", ");

      // First get the varname:
      while (token.hasMoreTokens())
      {
         String varName = token.nextToken();

         // System.out.println(varName);
         if (!varName.equals("DT"))
         {
            varNames.add(varName);
         }
      }

      // Next read the next line to get DT:
      line = dataStream.readASCIILine();
      token = new StringTokenizer(line, ", ");

      String DTString = token.nextToken();
      recordDT = Double.parseDouble(DTString);

      // System.out.println("DT = " + recordDT);


      boolean skipFirstOne = true;

      // Next read all the data and put it in the array Lists:
      while (!(line == null))
      {
         double[] dataForThisTick = new double[varNames.size()];
         int point = 0;

         // System.out.println(line);
         token = new StringTokenizer(line, ", ");

         // Ignore the first one:
         if (skipFirstOne)
         {
            System.out.println(token.nextToken());
            skipFirstOne = false;
         }

         while (token.hasMoreTokens())
         {
            String valString = token.nextToken();
            double value = Double.parseDouble(valString);

            // System.out.print(value + " ");
            dataForThisTick[point] = (value);
            point++;
         }

         dataArrays.add(dataForThisTick);
         line = dataStream.readASCIILine();
      }

      // Now we have all the data, now we need to put it into the dataBuffer:
      nPoints = dataArrays.size();
      dataBuffer.clearAll(nPoints);


      for (int i = 0; i < varNames.size(); i++)
      {
         String varName = varNames.get(i);



         DataBufferEntry newEntry = getDataBufferEntry(varName, dataBuffer, rootRegistryToAddNewVariablesTo, newVars);



         double[] someData = newEntry.getData();

         for (int j = 0; j < nPoints; j++)
         {
            someData[j] = (dataArrays.get(j))[i];    // Gets the ith element of the jth array to rearrange the data.
         }

         newEntry.setData(someData, nPoints);
      }

      dataStream.close();
      System.out.println("nPoints = " + nPoints);
      System.out.println("recordDT equals: " + recordDT);
      dataBuffer.setInPoint(0);
      dataBuffer.setOutPoint(nPoints - 1);
      dataBuffer.setIndex(0);

//    }
//    catch (IOException e){return -1;}
      return nPoints;
   }


   public void readState(YoVariableList varList, boolean printErrorForMissingVariables) throws IOException
   {
      readState(varList, false, printErrorForMissingVariables, null);
   }

   public void readState(YoVariableList varList, boolean createMissingVariables, boolean printErrorForMissingVariables, YoVariableRegistry registry) throws IOException
   {
      BufferedReader in;
      if (inFile.getName().endsWith(".gz"))
         in = new BufferedReader(new InputStreamReader(new BufferedInputStream(new GZIPInputStream(new BufferedInputStream(new FileInputStream(inFile))))));
      else
         in = new BufferedReader(new InputStreamReader(new FileInputStream(inFile)));

      String line;
      while ((line = in.readLine()) != null)
      {
         int equalsIndex = getEqualsIndex(line);
         int semiIndex = semiColonIndex(line);

         String varName = line.substring(0, equalsIndex).trim();
         String varVal = line.substring(equalsIndex + 1, semiIndex).trim();

         YoVariable<?> variable = varList.getVariable(varName);

         boolean variableNotFound = (variable == null);
         if (variableNotFound)
         {
            if (createMissingVariables)
            {
               NameSpace nameSpace = NameSpace.createNameSpaceFromAFullVariableName(varName);
               if (nameSpace != null)
               {
                  YoVariableRegistry registryToUse = registry.getOrCreateAndAddRegistry(nameSpace);
                  if (registryToUse == null)
                  {
                     if (printErrorForMissingVariables) System.err.println("Warning! Couldn't find an appropriate registry to use. Will just use " + registry);
                     registryToUse = registry;
                  }

                  varName = NameSpace.stripOffNameSpaceToGetVariableName(varName);
                  variable = new DoubleYoVariable(varName, registryToUse);
               }
               else
               {
                  variable = new DoubleYoVariable(varName, registry);
               }

               varList.addVariable(variable);
            }
            else
            {
               //TODO: JEP 10/23/2012:
               //Had to just print error instead of throw exception since DRCDemo0 
               // was somehow saving variables but not needed them on loading?
               if (printErrorForMissingVariables) System.err.println("Couldn't find variable " + varName + " in line:  " + line);
               continue;
//               throw new RuntimeException("Couldn't find variable " + varName + " in line:  " + line);
            }
         }

         variable.setValueFromDouble(Double.valueOf(varVal).doubleValue());

      }

      in.close();
   }

   private int semiColonIndex(String line)
   {
      int semiIndex = line.indexOf(";");
      if (semiIndex < 0)
         throw new RuntimeException("State File Fomat Error.  No ; in line:  " + line);

      return semiIndex;
   }

   private static int getEqualsIndex(String line)
   {
      int equalsIndex = line.indexOf("=");
      if (equalsIndex < 0)
         throw new RuntimeException("State File Fomat Error.  No = in line:  " + line);

      return equalsIndex;
   }



}
