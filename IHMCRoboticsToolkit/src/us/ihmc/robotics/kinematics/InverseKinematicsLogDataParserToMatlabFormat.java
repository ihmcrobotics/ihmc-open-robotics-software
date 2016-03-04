package us.ihmc.robotics.kinematics;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;

public class InverseKinematicsLogDataParserToMatlabFormat
{
   private static PrintStream dataToMatlab;

   public static void main(String[] s) throws IOException
   {
      try
      {
         dataToMatlab = new PrintStream(new File("dataToMatlabRightArm.txt"));
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }

      String fileName = "C:\\Users\\Wessel\\Desktop\\right_arm_joint_log.txt";


      BufferedReader reader = new BufferedReader(new FileReader(fileName));

      int k = 0;
      while (true)
      {
         String line = reader.readLine();
         if (line == null)
            break;
         String words[] = line.split(" ");

         Data data = new Data();
         data.type = words[0];
         data.side = words[1];
         data.time = Double.parseDouble(words[2]);

         if (data.type.equals("actual"))
         {
            k++;
         }

         for (int i = 0; i < 6; i++)
         {
            data.joints[i].joint = words[3 + i * 2];
            data.joints[i].value = Double.parseDouble(words[4 + i * 2]);

            if (data.type.equals("actual") && (k % 10 == 0))
            {
               dataToMatlab.println("actual(" + (k / 10) + "," + (i + 1) + ")=" + data.joints[i].value + ";");
            }

            if (data.type.equals("desired") && (k % 10 == 0))
            {
               dataToMatlab.println("desired(" + (k / 10) + "," + (i + 1) + ")=" + data.joints[i].value + ";");
            }
         }

      }

      System.out.println("Done");
   }

   public static class Data
   {
      String type;
      String side;
      double time;

      JointData joints[] = new JointData[6];

      public Data()
      {
         for (int i = 0; i < joints.length; i++)
         {
            joints[i] = new JointData();
         }
      }
   }


   public static class JointData
   {
      String joint;
      double value;
   }
}
