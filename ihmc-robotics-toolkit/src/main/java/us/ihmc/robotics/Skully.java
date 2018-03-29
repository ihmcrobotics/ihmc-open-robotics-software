package us.ihmc.robotics;

import us.ihmc.commons.PrintTools;

/**
 * This is Skully the friendly debugging skull.
 */
public class Skully
{
   private static final String face =
         "  ,----._\n" +
         " )\\___/  \\\n" +
         "/__, ,__,(|\n" +
         "|.d/ \\b. _/\n" +
         " \\/''  \\||\n" +
         "  '+++'//\n" +
         "  `-.-'";

   public Skully()
   {
      throw new RuntimeException("Skully says: 'I am a staic entity you can not create me.'");
   }

   public static void say(String message)
   {
      print("'" + message + "'\n" + face);
   }

   public static void say(String message, String additionalInfo)
   {
      print("'" + message + "'\n" + face + "\n" + additionalInfo);
   }

   private static void print(String output)
   {
      PrintTools.warn("Skully says he discovered something mysterious:\n   " + output.replace("\n", "\n   "));
   }
}
