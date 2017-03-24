package us.ihmc.simulationconstructionsettools.tools;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataFileReader;
import us.ihmc.simulationconstructionset.DataFileWriter;

public class BinaryToASCIIDataFileConverter
{
   private final String binaryFilename;
   private final String asciiFilename;

   public BinaryToASCIIDataFileConverter(String binaryFilename, String asciiFilename)
   {
      this.binaryFilename = binaryFilename;
      this.asciiFilename = asciiFilename;
   }

   public void convertData() throws IOException
   {
      File binaryFile = new File(binaryFilename);
      File asciiFile = new File(asciiFilename);

      int bufferSize = 16000;

      DataFileReader reader = new DataFileReader(binaryFile);

      YoVariableList newVars = new YoVariableList("Converter");

      DataBuffer dataBuffer = new DataBuffer(bufferSize);

      YoVariableRegistry rootRegistry = new YoVariableRegistry("root");

      reader.readData(newVars, rootRegistry, dataBuffer);

      DataFileWriter dataFileWriter = new DataFileWriter(asciiFile);

      boolean binary = false;
      boolean compress = false;

      ArrayList<YoVariable<?>> varsToWrite = newVars.getVariables();

      dataFileWriter.writeData("ConvertedData", 0.001, dataBuffer, varsToWrite, binary, compress);
   }

   public static void main(String[] args)
   {
      if (args.length != 2)
         throw new RuntimeException("Need two arguments");
      BinaryToASCIIDataFileConverter convert = new BinaryToASCIIDataFileConverter(args[0], args[1]);
      try
      {
         convert.convertData();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }
}
