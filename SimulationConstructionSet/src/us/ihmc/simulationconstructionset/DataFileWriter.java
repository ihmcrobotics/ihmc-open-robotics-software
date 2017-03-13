package us.ihmc.simulationconstructionset;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.DataOutput;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Date;
import java.util.zip.GZIPOutputStream;

import com.jmatio.io.MatFileIncrementalWriter;
import com.jmatio.types.MLDouble;
import com.jmatio.types.MLStructure;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

public class DataFileWriter
{
   @SuppressWarnings("unused")
   private static final boolean DEBUG = true;

   private final File outFile;

   public DataFileWriter(File file)    // String filename)
   {
      // this.filename = filename;
      this.outFile = file;
   }


/*   public void writeData(String model, double recordDT, DataBuffer dataBuffer)
   {
     ArrayList entries = dataBuffer.getEntries();

     try
       {
         //System.out.println("Creating output Streams");
         //FileOutputStream outStream = new FileOutputStream(outFile);
         //DataOutputStream dataStream = new DataOutputStream(outStream);

         DataOutputStream dataStream = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)));

         dataStream.writeBytes("$BEGIN_HEADER\n");

         //Calendar cal = Calendar.getInstance();
         Date today = new Date();
         //cal.setTime(today);
         //cal.get(cal.DATE);

         //dataStream.write
         dataStream.writeBytes("$WHEN " + today.toString() + "\n");
         dataStream.writeBytes("$MODEL " + model + "\n");

         dataStream.writeBytes("$INDIVIDUAL\n");
         dataStream.writeBytes("$SUN_DATA\n");
         dataStream.writeBytes("$BINARY\n");
         dataStream.writeBytes("$COLUMN\n");

         dataStream.writeBytes("$DT " + String.valueOf(recordDT) + "\n");
         dataStream.writeBytes("$NVAR " + String.valueOf(entries.size()) + "\n");

         for(int i=0;i<entries.size();i++)
         {
           DataBufferEntry entry = (DataBufferEntry) entries.get(i);
           YoVariable variable = entry.getVariable();
           dataStream.writeBytes("$VAR " + variable.getName() + " " + entry.getManualMinScaling() + " " + entry.getManualMaxScaling() + "\n");
         }

         int bufferLength = dataBuffer.getBufferInOutLength();

         dataStream.writeBytes("$N " + String.valueOf(bufferLength) + "\n");
         dataStream.writeBytes("$END_HEADER\n");


         // Write the binary data here:

         for(int i=0;i<entries.size();i++)
         {
           DataBufferEntry entry = (DataBufferEntry) entries.get(i);

           double[] data = entry.getWindowedData(dataBuffer.getInPoint(), dataBuffer.getOutPoint(), bufferLength);

           for(int j=0;j<bufferLength;j++)
           {
             dataStream.writeFloat(((float) data[j]));
           }
         }


         dataStream.close();
         //outStream.close();

       }
       catch (IOException e){}
     }



   public void writeState(String model, double recordDT, ArrayList variables)
   {
     //ArrayList variables = combinedVarPanel.getAllVars();

     try
       {
         //System.out.println("Creating output Streams");
         BufferedWriter out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(outFile)));

         for(int i=0;i<variables.size();i++)
         {
           YoVariable variable = (YoVariable) variables.get(i);
           out.write(variable.getName() + " = " + variable.getDoubleValue() + ";\n");
         }

         out.close();
       }
       catch (IOException e){}
     }

*/

   public void writeData(String model, double recordDT, DataBuffer dataBuffer, ArrayList<YoVariable<?>> vars, boolean binary, boolean compress)
   {
      writeData(model, recordDT, dataBuffer, vars, binary, compress, null);
   }

   public void writeData(String model, double recordDT, DataBuffer dataBuffer, ArrayList<YoVariable<?>> vars, boolean binary, boolean compress, Robot robot)
   {
      if (binary)
         writeBinaryData(model, recordDT, dataBuffer, vars, compress, robot);
      else
      {
         writeASCIIData(model, recordDT, dataBuffer, vars, compress);
      }
      
   }


   public void writeState(String model, double recordDT, ArrayList<YoVariable<?>> variables, boolean binary, boolean compress)
   {
      if (binary)
         writeBinaryState(model, recordDT, variables, compress);
      else
         writeASCIIState(model, recordDT, variables, compress);
   }

   private DataOutputStream openDataOutputStreamForWriting(File outFile, boolean compress) throws IOException
   {
      // System.out.println("Creating output Streams");
      // FileOutputStream outStream = new FileOutputStream(outFile);
      // DataOutputStream dataStream = new DataOutputStream(outStream);

      DataOutputStream dataStream;
      if (compress)
         dataStream = new DataOutputStream(new BufferedOutputStream(new GZIPOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)))));
      else
         dataStream = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)));

      return dataStream;
   }

   private void writeHeaderInformation(DataOutput dataOutputStream, ArrayList<DataBufferEntry> entries, String model, double recordDT, DataBuffer dataBuffer,
           ArrayList<YoVariable<?>> vars, boolean compress, Robot robot)
           throws IOException
   {
      String columnFormatted = "$COLUMN";
      int bufferLength = dataBuffer.getBufferInOutLength();

      writeHeaderInformation(dataOutputStream, entries, model, columnFormatted, recordDT, bufferLength, vars, compress, robot);
   }

   private void writeHeaderInformation(DataOutput dataOutputStream, ArrayList<DataBufferEntry> entries, String model, String columnOrRowFormatted,
           double recordDT, int bufferLength, ArrayList<YoVariable<?>> vars, boolean compress, Robot robot)
           throws IOException
   {
//    if (DEBUG) System.out.println("Writing out $BEGIN_HEADER");

      dataOutputStream.writeBytes("$BEGIN_HEADER\n");

      // Calendar cal = Calendar.getInstance();
      Date today = new Date();

      // cal.setTime(today);
      // cal.get(cal.DATE);

      // dataStream.write
      dataOutputStream.writeBytes("$WHEN " + today.toString() + "\n");
      dataOutputStream.writeBytes("$MODEL " + model + "\n");

      dataOutputStream.writeBytes("$INDIVIDUAL\n");
      dataOutputStream.writeBytes("$SUN_DATA\n");
      dataOutputStream.writeBytes("$BINARY\n");
      dataOutputStream.writeBytes(columnOrRowFormatted + "\n");

//    dataOutputStream.writeBytes("$COLUMN\n");


      dataOutputStream.writeBytes("$DT " + String.valueOf(recordDT) + "\n");

      int nVars = 0;

      if (entries != null)
      {
         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();

            if (vars.contains(variable))
               nVars++;
         }
      }
      else
      {
         nVars = vars.size();
      }

      dataOutputStream.writeBytes("$NVAR " + nVars + "\n");    // +++JEP:  Need to know how many variables are valid in the list!!!

      if (entries != null)
      {
         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();

            if (vars.contains(variable))
            {
//             dataOutputStream.writeBytes("$VAR " + variable.getName() + " " + entry.getManualMinScaling() + " " + entry.getManualMaxScaling() + "\n");
               dataOutputStream.writeBytes("$VAR " + variable.getFullNameWithNameSpace() + " " + entry.getManualMinScaling() + " "
                                           + entry.getManualMaxScaling() + "\n");
            }
         }
      }
      else
      {
         for (int i = 0; i < vars.size(); i++)
         {
            YoVariable<?> variable = vars.get(i);

//          dataOutputStream.writeBytes("$VAR " + variable.getName() + " -1.0 1.0\n");
            dataOutputStream.writeBytes("$VAR " + variable.getFullNameWithNameSpace() + " -1.0 1.0\n");
         }

      }

      dataOutputStream.writeBytes("$N " + String.valueOf(bufferLength) + "\n");

      if (robot != null)
      {
         RobotDefinitionFixedFrame rd = new RobotDefinitionFixedFrame();
         rd.createRobotDefinitionFromRobot(robot);
         String robotConfig = "$" + rd.toString();
         robotConfig = replaceAll(robotConfig, "\n", "\n$");
         robotConfig = robotConfig.substring(0, robotConfig.length() - 1);
         dataOutputStream.writeBytes(robotConfig + "\n");
      }
      else
      {
         System.err.println("Warning: Could not write robot definition data: Robot is null");
      }

      dataOutputStream.writeBytes("$END_HEADER\n");
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

   public DataOutputStream openDataOutputStreamAndWriteHeaderInformationForLoggingData(String model, double recordDT, DataBuffer dataBuffer,
           ArrayList<YoVariable<?>> vars, boolean compress)    // , Robot robot
   {
      return openDataOutputStreamAndWriteHeaderInformationForLoggingData(model, recordDT, dataBuffer, vars, compress, null);
   }

   private DataOutputStream openDataOutputStreamAndWriteHeaderInformationForLoggingData(String model, double recordDT, DataBuffer dataBuffer,
           ArrayList<YoVariable<?>> vars, boolean compress, Robot robot)
   {
      DataOutputStream dataOutputStream = null;

      try
      {
         dataOutputStream = openDataOutputStreamForWriting(outFile, compress);
         ArrayList<DataBufferEntry> entries = null;

         if (dataBuffer != null)
            entries = dataBuffer.getEntries();

         String columnFormatted = "$ROW";
         int bufferLength = -1;    // +++JEP080725: Not sure how many points we will log, so we will set it to -1. The reader then needs to know to go to the end of the file! // dataBuffer.getBufferInOutLength();

         writeHeaderInformation(dataOutputStream, entries, model, columnFormatted, recordDT, bufferLength, vars, compress, robot);
      }

      catch (IOException ioException)
      {
         System.err.println("Caught IOException in openDataOutputStreamAndWriteHeaderInformationForLoggingData. exception = " + ioException);
      }

      return dataOutputStream;
   }

   public void writeOutOneRowOfLogData(DataOutput dataOutputStream, ArrayList<YoVariable<?>> variablesToWrite) throws IOException
   {
      for (YoVariable<?> variableToWrite : variablesToWrite)
      {
         double value = variableToWrite.getValueAsDouble();
         dataOutputStream.writeFloat((float) value);
      }
   }

   public void writeOutOneRowOfLogData(DataOutput dataOutputStream, double[] dataToWrite, int numberOfVariables) throws IOException
   {
      if (numberOfVariables > dataToWrite.length)
         throw new RuntimeException("numberOfVariables > dataToWrite.length");

      for (int i = 0; i < numberOfVariables; i++)
      {
         double value = dataToWrite[i];
         dataOutputStream.writeFloat((float) value);
      }
   }
   
   
   public void writeMatlabBinaryData(double recordDT, DataBuffer dataBufferSortedByNamespace, ArrayList<YoVariable<?>> vars)
   {
      MatFileIncrementalWriter writer;
      try
      {
         writer = new MatFileIncrementalWriter(outFile);

         int bufferLength = dataBufferSortedByNamespace.getBufferInOutLength();
         ArrayList<DataBufferEntry> entries = dataBufferSortedByNamespace.getEntries();
         
         MLDouble dt = new MLDouble("DT", new double[][]{{recordDT}});
         writer.write(dt);
         

         MLStructure mlRoot=null, mlNode;
         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();
            
            ArrayList<String> subNames = variable.getNameSpace().getSubNames();
            int subNameDepth= 0;

            if (vars.contains(variable))
            {

               //find/create root
               String rootName = subNames.get(subNameDepth++);
               if(mlRoot==null)
               {
                  mlRoot = new MLStructure(rootName, new int[]{1,1});
               }
               else 
               {
                  if(!mlRoot.getName().equals(rootName)) 
                  {
                          writer.write(mlRoot);
                          PrintTools.info(this, "MLStructure '"+ mlRoot.getName() + "' written", true);
                          mlRoot = new MLStructure(rootName, new int[]{1,1});
                  }
               }
               
               //query/create node
               mlNode = mlRoot;
               while(subNameDepth < subNames.size())
               {
                  String childSubName = subNames.get(subNameDepth);
                  MLStructure mlSubNode = (MLStructure)mlNode.getField(childSubName);
                  if(mlSubNode==null)
                  {
                     mlSubNode = new MLStructure(childSubName, new int[]{1,1});
                     mlNode.setField(childSubName, mlSubNode);
                  }
                  mlNode=mlSubNode;
                  subNameDepth++;
               }

               //store yo-variable as a new field
               double[] data = entry.getWindowedData(dataBufferSortedByNamespace.getInPoint(), bufferLength);
               MLDouble outArray = new MLDouble(variable.getName(), new int[] { 1, bufferLength });
               for (int j = 0; j < bufferLength; j++)
               {
                  ((MLDouble)outArray).set(data[j], j);
               }

               mlNode.setField(variable.getName(), outArray);
            }
         }
         if (mlRoot != null)
         {
            writer.write(mlRoot);
            PrintTools.info(this, "MLStructure '"+ mlRoot.getName() + "' written", true);
         }
         writer.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   

   private void writeBinaryData(String model, double recordDT, DataBuffer dataBuffer, ArrayList<YoVariable<?>> vars, boolean compress, Robot robot)
   {
      try
      {
         DataOutputStream dataOutputStream = openDataOutputStreamForWriting(outFile, compress);
         ArrayList<DataBufferEntry> entries = dataBuffer.getEntries();
         writeHeaderInformation(dataOutputStream, entries, model, recordDT, dataBuffer, vars, compress, robot);

         int bufferLength = dataBuffer.getBufferInOutLength();

         // Write the binary data here:

         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();

            if (vars.contains(variable))
            {
               double[] data = entry.getWindowedData(dataBuffer.getInPoint(), /* dataBuffer.getOutPoint(), */ bufferLength);

               for (int j = 0; j < bufferLength; j++)
               {
                  dataOutputStream.writeFloat(((float) data[j]));
               }
            }
         }

         dataOutputStream.close();
      }
      catch (IOException e)
      {
      }
   }

   private void writeASCIIData(String model, double recordDT, DataBuffer dataBuffer, ArrayList<YoVariable<?>> vars, boolean compress)
   {
      ArrayList<DataBufferEntry> entries = dataBuffer.getEntries();

      try
      {
         // System.out.println("Creating output Streams");

         PrintStream printStream;

         // FileOutputStream outStream = new FileOutputStream(outFile);

         if (compress)
            printStream = new PrintStream(new BufferedOutputStream(new GZIPOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)))));
         else
            printStream = new PrintStream(new BufferedOutputStream(new FileOutputStream(outFile)));

         // DataOutputStream dataStream = new DataOutputStream(outStream);

         // Write the data here:
         int bufferLength = dataBuffer.getBufferInOutLength();

         // Names of the variables
         String varnamesToWrite[] = new String[vars.size()];

         // Data of the variables
         double[][] dataToWrite = new double[vars.size()][];



         // Find the matching variables
         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();

            if (vars.contains(variable))
            {
               varnamesToWrite[vars.indexOf(variable)] = entry.getVariable().getFullNameWithNameSpace();
               varnamesToWrite[vars.indexOf(variable)] = varnamesToWrite[vars.indexOf(variable)].replace("[", "");
               varnamesToWrite[vars.indexOf(variable)] = varnamesToWrite[vars.indexOf(variable)].replace("]", "");

               double[] data = entry.getWindowedData(dataBuffer.getInPoint(), /* dataBuffer.getOutPoint(), */ bufferLength);
               dataToWrite[vars.indexOf(variable)] = data;
            }
         }

         // Write the data
         printStream.println("DT = " + recordDT + ";");

         for (int i = 0; i < varnamesToWrite.length; i++)
         {
            printStream.print(varnamesToWrite[i] + " = [");

            for (int j = 0; j < bufferLength; j++)
            {
               // dataStream.writeFloat(((float) data[j]));
               double dataElement =dataToWrite[i][j];
            	printStream.print( dataElement+ " ");
            }

            printStream.println("];");
         }


         printStream.close();

         // outStream.close();
      }
      catch (IOException e)
      {
      }
   }

   public void writeSpreadsheetFormattedData(DataBuffer dataBuffer, ArrayList<? extends YoVariable<?>> vars)
   {
      ArrayList<DataBufferEntry> entries = dataBuffer.getEntries();

      try
      {
         // System.out.println("Creating output Streams");

         PrintStream printStream;

         // FileOutputStream outStream = new FileOutputStream(outFile);

         printStream = new PrintStream(new BufferedOutputStream(new FileOutputStream(outFile)));

         // Write the data here:
         int bufferLength = dataBuffer.getBufferInOutLength();

         // Names of the variables
         String varnamesToWrite[] = new String[vars.size()];

         // Data of the variables
         double[][] dataToWrite = new double[vars.size()][];

         // Find the matching variables
         for (int i = 0; i < entries.size(); i++)
         {
            DataBufferEntry entry = entries.get(i);
            YoVariable<?> variable = entry.getVariable();

            if (vars.contains(variable))
            {
               varnamesToWrite[vars.indexOf(variable)] = entry.getVariable().getFullNameWithNameSpace();

               double[] data = entry.getWindowedData(dataBuffer.getInPoint(), /* dataBuffer.getOutPoint(), */ bufferLength);
               dataToWrite[vars.indexOf(variable)] = data;
            }
         }


         // Write the variable names:
//       printStream.print("DT,");

         for (int i = 0; i < varnamesToWrite.length; i++)
         {
            printStream.print(varnamesToWrite[i]);
            if (i < varnamesToWrite.length - 1)
               printStream.print(",");
            else
               printStream.println("");
         }

         // Write the data:
         for (int j = 0; j < bufferLength; j++)
         {
//          if (j == 0)
//             printStream.print(recordDT + ",");
//          else
//             printStream.print(",");

            for (int i = 0; i < dataToWrite.length; i++)
            {
               double[] data = dataToWrite[i];
               printStream.print(data[j]);
               if (i < dataToWrite.length - 1)
                  printStream.print(",");
               else
                  printStream.println("");
            }
         }

         printStream.close();

         // outStream.close();
      }
      catch (IOException e)
      {
      }

   }

   private void writeBinaryState(String model, double recordDT, ArrayList<YoVariable<?>> variables, boolean compress)
   {
      try
      {
         DataOutputStream dataStream;
         if (compress)
            dataStream = new DataOutputStream(new BufferedOutputStream(new GZIPOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)))));
         else
            dataStream = new DataOutputStream(new BufferedOutputStream(new FileOutputStream(outFile)));

         dataStream.writeBytes("$BEGIN_HEADER\n");

         // Calendar cal = Calendar.getInstance();
         Date today = new Date();

         // cal.setTime(today);
         // cal.get(cal.DATE);

         // dataStream.write
         dataStream.writeBytes("$WHEN " + today.toString() + "\n");
         dataStream.writeBytes("$MODEL " + model + "\n");

         dataStream.writeBytes("$INDIVIDUAL\n");
         dataStream.writeBytes("$SUN_DATA\n");
         dataStream.writeBytes("$BINARY\n");
         dataStream.writeBytes("$COLUMN\n");

         dataStream.writeBytes("$DT " + String.valueOf(recordDT) + "\n");
         dataStream.writeBytes("$NVAR " + String.valueOf(variables.size()) + "\n");

         for (int i = 0; i < variables.size(); i++)
         {
            YoVariable<?> variable = variables.get(i);

            dataStream.writeBytes("$VAR " + variable.getFullNameWithNameSpace() + " 1.0 1.0 " + "\n");

//          dataStream.writeBytes("$VAR " + variable.getName() + " 1.0 1.0 " + "\n");
         }

         dataStream.writeBytes("$N 1" + "\n");
         dataStream.writeBytes("$END_HEADER\n");

         // Write the binary data here:

         for (int i = 0; i < variables.size(); i++)
         {
            YoVariable<?> variable = variables.get(i);

            dataStream.writeFloat(((float) variable.getValueAsDouble()));
         }

         dataStream.close();
      }
      catch (IOException e)
      {
      }
   }

   private void writeASCIIState(String model, double recordDT, ArrayList<YoVariable<?>> variables, boolean compress)
   {
      try
      {
         BufferedWriter out;

         if (compress)
            out = new BufferedWriter(
                new OutputStreamWriter(new BufferedOutputStream(new GZIPOutputStream(new BufferedOutputStream(new FileOutputStream(outFile))))));
         else
            out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(outFile)));


         for (YoVariable<?> variable : variables)
         {
            String nameSpaceName = variable.getYoVariableRegistry().getNameSpace().getName();
            String variableName = variable.getName();
            String variableValue = variable.getNumericValueAsAString();
            out.write(nameSpaceName + "." + variableName + " = " + variableValue + ";\n");
         }

         out.close();
      }
      catch (IOException e)
      {
      }
   }

   public void writeSpreadsheetFormattedState(DataBuffer dataBuffer, ArrayList<? extends YoVariable<?>> vars)
   {
      try
      {
         BufferedWriter out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(outFile)));

         for (YoVariable<?> variable : vars)
         {
            out.write(variable.getFullNameWithNameSpace());

            boolean lastVariable = (variable == vars.get(vars.size() - 1));
            if (!lastVariable)
               out.write(",");
         }

         out.write("\n");

         for (YoVariable<?> variable : vars)
         {
            String variableValue = variable.getNumericValueAsAString();
            out.write(variableValue);

            boolean lastVariable = (variable == vars.get(vars.size() - 1));
            if (!lastVariable)
               out.write(",");
         }

         out.close();
      }
      catch (IOException e)
      {
      }
   }
}
