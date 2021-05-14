package us.ihmc.tools.processManagement;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Properties;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.lang3.SystemUtils;

public class ProcessTools
{
   public static ArrayList<String> getAllSystemProcesses()
   {
      try
      {
         Process psSystemCommand = null;

         if (SystemUtils.IS_OS_LINUX || SystemUtils.IS_OS_MAC)
            psSystemCommand = Runtime.getRuntime().exec("ps -e");
         else if (SystemUtils.IS_OS_WINDOWS)
            psSystemCommand = Runtime.getRuntime().exec(System.getenv("windir") + "\\system32\\" + "tasklist.exe");

         BufferedReader reader = new BufferedReader(new InputStreamReader(psSystemCommand.getInputStream()));

         String line;
         ArrayList<String> processLines = new ArrayList<>();
         while ((line = reader.readLine()) != null)
            processLines.add(line);
         
         return processLines;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         
         return null;
      }
   }

   public static String[] getCurrentJVMProperties()
   {
      Properties properties = System.getProperties();
      String[] propertyStrings = new String[properties.size()];
      int i = 0;
      for (String stringPropertyName : properties.stringPropertyNames())
      {
         propertyStrings[i] = "-D" + stringPropertyName + "=" + System.getProperty(stringPropertyName);
         ++i;
      }
      return propertyStrings;
   }

   public static String[] constructJavaProcessCommand(String javaHome,
                                                      String nativeLibraryPath,
                                                      String classpath,
                                                      Class<?> mainClass,
                                                      String[] javaArgs,
                                                      String[] programArgs)
   {
      String[] spawnString = new String[] {javaHome + "/bin/java"};

      if (javaArgs != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, javaArgs);
      }

      if (nativeLibraryPath != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, "-Djava.library.path=" + nativeLibraryPath);
      }

      if (classpath != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, "-cp", classpath);
      }

      spawnString = ArrayUtils.addAll(spawnString, mainClass.getCanonicalName());

      if (programArgs != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, programArgs);
      }

      return spawnString;
   }

   public static PrintStream createJansiFilteredStream(Path outputFile) throws IOException
   {
      return new PrintStream(Files.newOutputStream(outputFile));
   }

//   public static PrintStream createJansiFilteredStream(OutputStream outputStream) throws IOException
//   {
//      return new AnsiProcessor(outputStream);
//      if (SystemUtils.IS_OS_WINDOWS)
//      {
//         return new WindowsAnsiPrintStream(new PrintStream(outputStream));
//      }
//      else
//      {
//         return new AnsiPrintStream(new PrintStream(outputStream), true);
//      }
//   }
}
