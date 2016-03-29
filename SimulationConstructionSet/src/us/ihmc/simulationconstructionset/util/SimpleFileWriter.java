package us.ihmc.simulationconstructionset.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/*
* John Carff
* File Name: FileWritter.java
* Created on: Sep 18, 2007
*
 */
public class SimpleFileWriter
{
   File outputFile;
   FileWriter out;

   public SimpleFileWriter(File f)
   {
      outputFile = f;

      try
      {
         out = new FileWriter(outputFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void write(String outString)
   {
      try
      {
         out.write(outString);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void close()
   {
      try
      {
         if (out != null)
            out.close();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }
}
