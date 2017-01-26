package us.ihmc.tools.testing;

import java.awt.Desktop;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.charset.Charset;
import java.nio.file.FileVisitOption;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import org.pitest.mutationtest.commandline.MutationCoverageReport;

public class MutationTestingTools
{
   public static void doPITMutationTestAndOpenResult(String targetTests, String targetClasses)
   {
      final String reportDirectoryName = "pit-reports";
      MutationCoverageReport.main(new String[] {"--reportDir", reportDirectoryName,
                                                "--targetClasses", targetClasses,
                                                "--targetTests", targetTests,
                                                "--sourceDirs", "src,test",
                                                "--mutators",
                                                "ALL"
//                                                "RETURN_VALS,"
//                                              + "INLINE_CONSTS,"
//                                              + "MATH,"
//                                              + "VOID_METHOD_CALLS,"
//                                              + "NEGATE_CONDITIONALS,"
//                                              + "CONDITIONALS_BOUNDARY,"
//                                              + "INCREMENTS,"
//                                              + "REMOVE_INCREMENTS,"
//                                              + "NON_VOID_METHOD_CALLS,"
//                                              + "CONSTRUCTOR_CALLS,"
//                                              + "REMOVE_CONDITIONALS_EQ_IF,"
//                                              + "REMOVE_CONDITIONALS_EQ_ELSE,"
//                                              + "REMOVE_CONDITIONALS_ORD_IF,"
//                                              + "REMOVE_CONDITIONALS_ORD_ELSE,"
//                                              + "REMOVE_CONDITIONALS,"
//                                              + "EXPERIMENTAL_MEMBER_VARIABLE,"
//                                              + "EXPERIMENTAL_SWITCH,"
//                                              + "EXPERIMENTAL_ARGUMENT_PROPAGATION,"
//                                              + "REMOVE_SWITCH"
                                                });

      
      File reportDirectory = new File(reportDirectoryName);
      if (reportDirectory.isDirectory() && reportDirectory.exists())
      {
         String[] list = reportDirectory.list();
         final String lastDirectoryName = list[list.length-1];

         System.out.println("Found last directory " + lastDirectoryName);
         
         walkFlat(Paths.get(reportDirectoryName, lastDirectoryName), new BasicPathVisitor()
         {
            @Override
            public FileVisitResult visitPath(Path path, PathType pathType)
            {
               String longPathName = path.getFileName().toString();
               if (longPathName.length() > 50)
               {
                  String newPathName = longPathName.substring(0, 20) + "..." + longPathName.substring(longPathName.length() - 20, longPathName.length());
                  Path newPath = Paths.get(reportDirectoryName, lastDirectoryName, newPathName);
                  
                  Path indexPath = Paths.get(reportDirectoryName, lastDirectoryName, "index.html");
                  List<String> lines = readAllLines(indexPath);
                  ArrayList<String> newLines = new ArrayList<>();
                  for (String originalLine : lines)
                  {
                     newLines.add(originalLine.replaceAll(longPathName, newPathName));
                  }
                  writeAllLines(newLines, indexPath);
                  
                  try
                  {
                     Files.move(path, newPath);
                  }
                  catch (IOException e)
                  {
                     e.printStackTrace();
                  }
               }
               return FileVisitResult.CONTINUE;
            }
         });

         File reportFile = new File(reportDirectory, lastDirectoryName + "/index.html");
         String absolutePath;
         try
         {
            absolutePath = reportFile.getCanonicalPath();

            absolutePath = absolutePath.replace("\\", "/");
            System.out.println("Opening " + "file://" + absolutePath);

            URI uri = new URI("file://" + absolutePath);
            Desktop.getDesktop().browse(uri);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
      }
   }
   
   public static void doPITMutationTestAndOpenResult(Class<?> clazz)
   {
      String targetTests = clazz.getName();
      String targetClassesInSamePackage = MutationTestingTools.createClassSelectorStringFromTargetString(targetTests);
      doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }

   public static String createClassSelectorStringFromTargetString(String targetTests)
   {
      return targetTests.substring(0, targetTests.lastIndexOf('.')) + "*";
   }
   
   // COPIED FROM IHMCJAVATOOLKIT FOR DEPENDENCY REASONS
   
   private static enum PathType
   {
      FILE, DIRECTORY,
   }
   
   private static abstract class BasicPathVisitor
   {
      public FileVisitResult visitPath(Path path, PathType pathType)
      {
         return FileVisitResult.CONTINUE;
      }
   }
   
   private static void walkDepth(final Path directory, int maxDepth, final BasicPathVisitor basicFileVisitor)
   {
      try
      {
         Files.walkFileTree(directory, EnumSet.noneOf(FileVisitOption.class), maxDepth, new SimpleFileVisitor<Path>()
         {
            @Override
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) throws IOException
            {
               if (dir.equals(directory))
                  return FileVisitResult.CONTINUE;
               
               return basicFileVisitor.visitPath(dir, PathType.DIRECTORY);
            }
            
            @Override
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException
            {
               if (Files.isDirectory(file))
               {
                  return basicFileVisitor.visitPath(file, PathType.DIRECTORY);
               }
               {
                  return basicFileVisitor.visitPath(file, PathType.FILE);
               }
            }
         });
      }
      catch (IOException e)
      {
      }
   }

   private static void walkFlat(final Path directory, final BasicPathVisitor basicFileVisitor)
   {
      walkDepth(directory, 1, basicFileVisitor);
   }
   
   private static List<String> readAllLines(Path path)
   {
      try
      {
         return Files.readAllLines(path, Charset.forName("UTF-8"));
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
   
   private static void writeAllLines(List<String> lines, Path path)
   {
      PrintWriter printer = newPrintWriter(path);
      
      for (String line : lines)
      {
         printer.println(line);
      }
      
      printer.close();
   }
   
   private static PrintWriter newPrintWriter(Path path)
   {
      return newPrintWriter(path, false);
   }
   
   private static PrintWriter newPrintWriter(Path path, boolean append)
   {
      try
      {
         Writer outWriter = new FileWriter(path.toFile(), append);
         BufferedWriter bufferedWriter = new BufferedWriter(outWriter);
         return new PrintWriter(bufferedWriter); 
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
}

