package us.ihmc.sensorProcessing.pointClouds.parsing;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Scanner;

/**
 * @author Peter Abeles
 */
public class RosCloudToCvsFormat
{
   public static void main(String[] args) throws IOException
   {
      Scanner sc = new Scanner(new File("/home/pja/Downloads/a/headData_20131001162531/pointcloud.txt"));

      PrintStream out = new PrintStream("output.txt");
      int i=0;
      String lineOut = "";
      int lineNumber = 0;

      while(sc.hasNextLine()){

         if( lineNumber++ % 1000 == 0 )
            System.out.println("lines read "+lineNumber);

         String line = sc.nextLine();

         if( line.startsWith("      ") && line.charAt(7) == ':' ) {
            lineOut += line.substring(9);


            if( ++i == 3 ) {
               out.println(lineOut);
               lineOut = "";
               i = 0;
            } else {
               lineOut += " ";
            }
         }
      }

      out.close();
      System.out.println("Done!");
   }
}
