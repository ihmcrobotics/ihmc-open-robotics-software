package us.ihmc.robotics;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class EuclidCoreTestMissingTools
{
   public static String toStringFullPrecision(RigidBodyTransform rigidBodyTransform)
   {
      return EuclidCoreIOTools.getRigidBodyTransformString(EuclidCoreIOTools.getStringFormat(18, 18), rigidBodyTransform);
   }

   public static RigidBodyTransform newRigidBodyTransformFromString(String rigidBodyTransformAsString)
   {
      String[] lines = rigidBodyTransformAsString.split("\\R"); // Split the input by newlines [3]
      String[] tokens;
      double[] doubles = new double[lines.length * 4]; // Initialize an array to store the doubles
      int index = 0;

      for (String line : lines)
      {
         line = line.replace("|", "").trim(); // Remove the '|' character and trim whitespace
         tokens = line.split("\\s+"); // Split the line by whitespace [2]

         for (String token : tokens)
         {
            doubles[index++] = Double.parseDouble(token); // Convert the token to a double and store it in the array [1]
         }
      }

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      return new RigidBodyTransform(doubles);
   }
}
