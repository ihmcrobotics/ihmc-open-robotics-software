package us.ihmc.perception;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class MapsenseTools
{
   public static int loadPointCloud(String filePath, float[] pointsToPack) throws FileNotFoundException
   {
      Scanner scanner = new Scanner(new File(filePath));

      scanner.nextLine();
      int i = 0;
      while(scanner.hasNext())
      {
         String input = scanner.nextLine();
         String[] words = input.split(" ");

         //         System.out.println("Index:" + i + ":\t" + Arrays.toString(words));

         pointsToPack[i*3] = Float.parseFloat(words[0]);
         pointsToPack[i*3+1] = Float.parseFloat(words[1]);
         pointsToPack[i*3+2] = Float.parseFloat(words[2]);

         i++;

      }
      return i;
   }
}
