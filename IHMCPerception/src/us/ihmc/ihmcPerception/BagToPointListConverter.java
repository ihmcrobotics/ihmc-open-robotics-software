package us.ihmc.ihmcPerception;

import com.sun.org.apache.xml.internal.serialize.LineSeparator;

import javax.swing.*;
import java.io.*;

/**
 * Created by agrabertilton on 1/26/15.
 */
public class BagToPointListConverter
{

   public static void main(String[] args)
   {
      JFileChooser chooser = new JFileChooser();
      int returnVal = chooser.showOpenDialog(null);
      if (returnVal != JFileChooser.APPROVE_OPTION)
      {
         System.err.println("Can not load selected file: " + chooser.getName());
         return;
      }

      File fileIn = chooser.getSelectedFile();
      String fileName = fileIn.getName();
      fileName += ".pointLst";
      System.out.println(fileName);

      JFileChooser outputChooser = new JFileChooser();
      outputChooser.setDialogTitle("Specify a file to save");
      outputChooser.setSelectedFile(new File(fileName));
      returnVal = outputChooser.showSaveDialog(null);
      if (returnVal != JFileChooser.APPROVE_OPTION){
         System.err.println("Can not save here: " + chooser.getName());
         return;
      }
      File fileOut = outputChooser.getSelectedFile();

      try
      {
         BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(fileIn)));

         BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(fileOut)));

         String line;
         String[] numbers;
         while ((line = bufferedReader.readLine()) != null){
            numbers = line.split(" ", 5);
            bufferedWriter.write("(");
            for (int i = 1; i <= 3; i++){
               bufferedWriter.write(numbers[i]);
               if (i != 3)
                  bufferedWriter.write(", ");
            }
            bufferedWriter.write(")" + System.lineSeparator());
         }

         bufferedWriter.close();
         bufferedReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
