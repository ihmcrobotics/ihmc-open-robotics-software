package us.ihmc.tools.io.files;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.nio.CommonPaths;
import us.ihmc.commons.nio.FileToolsTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DeprecatedFileToolsTest
{
   private static final Path FILE_TOOLS_TEST_PATH = CommonPaths.deriveTestResourcesPath(FileToolsTest.class);
   private static final Path TEXT_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleTextFiles");
   private static final Path JAVA_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleJavaFiles");
   private static final Path EMPTY_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleEmptyFiles");
   
   private static final String EXAMPLE_FILE_1_TEXT_LINE_1 = "This is example File 1 !!&&#))(";
   private static final String EXAMPLE_FILE_2_TEXT_LINE_1 = "This is example File 2 *@&&%*@";
   private static final String EXAMPLE_FILE_2_TEXT_LINE_2 = "It has two lines";
   private static final String EXAMPLE_JAVA_FILE1_JAVA = "ExampleJavaFile1.java.txt";
   private static final String EXAMPLE_JAVA_FILE2_JAVA = "ExampleJavaFile2.java.txt";
   private static final String TEST_FILE_BAD_TXT = "testFileBad.txt";
   @SuppressWarnings("unused")
   private static final String FILE_TOOLS_EXAMPLE_FILE_CAT_TXT = "FileToolsExampleFileCat.txt";
   private static final String FILE_TOOLS_EXAMPLE_FILE1_TXT = "FileToolsExampleFile1.txt";
   private static final String FILE_TOOLS_EXAMPLE_FILE2_TXT = "FileToolsExampleFile2.txt";
   private static final String TEST_READ_ALL_LINES_TXT = "testReadAllLines.txt";
   
   private static final Path EXAMPLE_JAVA_FILE1_PATH = JAVA_DIRECTORY_PATH.resolve(EXAMPLE_JAVA_FILE1_JAVA);
   private static final Path EXAMPLE_JAVA_FILE2_PATH = JAVA_DIRECTORY_PATH.resolve(EXAMPLE_JAVA_FILE2_JAVA);
   private static final Path FILE_TOOLS_EXAMPLE_FILE1_PATH = TEXT_DIRECTORY_PATH.resolve(FILE_TOOLS_EXAMPLE_FILE1_TXT);
   private static final Path FILE_TOOLS_EXAMPLE_FILE2_PATH = TEXT_DIRECTORY_PATH.resolve(FILE_TOOLS_EXAMPLE_FILE2_TXT);
   private static final Path READ_ALL_LINES_PATH = FILE_TOOLS_TEST_PATH.resolve(TEST_READ_ALL_LINES_TXT);
   
   @Before
   public void setUp()
   {
      FileTools.ensureDirectoryExists(FILE_TOOLS_TEST_PATH);
      FileTools.ensureDirectoryExists(TEXT_DIRECTORY_PATH);
      FileTools.ensureDirectoryExists(JAVA_DIRECTORY_PATH);
      FileTools.ensureDirectoryExists(EMPTY_DIRECTORY_PATH);
      
      createJavaFile1();
      createJavaFile2();
      createTestFile1();
      createTestFile2();
      createReadAllLinesFile();
   }
   
   @After
   public void tearDown()
   {
      try
      {
         Files.delete(EXAMPLE_JAVA_FILE1_PATH);
         Files.delete(EXAMPLE_JAVA_FILE2_PATH);
         Files.delete(FILE_TOOLS_EXAMPLE_FILE1_PATH);
         Files.delete(FILE_TOOLS_EXAMPLE_FILE2_PATH);
         Files.delete(READ_ALL_LINES_PATH);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   private static void createTestFile1()
   {
      PrintWriter writer = FileTools.newPrintWriter(FILE_TOOLS_EXAMPLE_FILE1_PATH);
      writer.println(EXAMPLE_FILE_1_TEXT_LINE_1);
      writer.flush();
      writer.close();
   }

   private static void createTestFile2()
   {
      PrintWriter writer = FileTools.newPrintWriter(FILE_TOOLS_EXAMPLE_FILE2_PATH);
      writer.println(EXAMPLE_FILE_2_TEXT_LINE_1);
      writer.println(EXAMPLE_FILE_2_TEXT_LINE_2);
      writer.flush();
      writer.close();
   }

   private static void createJavaFile1()
   {
      PrintWriter writer = FileTools.newPrintWriter(EXAMPLE_JAVA_FILE1_PATH);
      writer.println("// This is a comment");
      writer.println("package us.ihmc.tools.io.files.fileToolsTest.exampleJavaFiles;");
      writer.println("public class ExampleJavaFile1");
      writer.println("{");
      writer.println("public static void main(String[] args)");
      writer.println("{");
      writer.println("System.out.println(\"Hello, World!\");");
      writer.println("}");
      writer.println("}");
      writer.println("// So is this");
      writer.flush();
      writer.close();
   }

   private static void createJavaFile2()
   {
      PrintWriter writer = FileTools.newPrintWriter(EXAMPLE_JAVA_FILE2_PATH);
      writer.println("package us.ihmc.tools.io.files.fileToolsTest.exampleJavaFiles;");
      writer.println("public class ExampleJavaFile2");
      writer.println("{");
      writer.println("public static void main(String[] args)");
      writer.println("{");
      writer.println("System.out.println(\"Hello, World!\");");
      writer.println("}");
      writer.println("}");
      writer.flush();
      writer.close();
   }
   
   private static void createReadAllLinesFile()
   {
      PrintWriter writer = FileTools.newPrintWriter(READ_ALL_LINES_PATH);
      writer.print("line1\r\nline2\nline3\r");
      writer.close();
   }
   
   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetBufferedReader()
   {
      File testFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH.toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         BufferedReader reader = DeprecatedFileTools.getFileReader(testFile1.getAbsolutePath());
         assertNotNull(reader);
         reader.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = FileNotFoundException.class)
   public void testGetBufferedReaderWithFileNotFoundException() throws FileNotFoundException
   {
      DeprecatedFileTools.getFileReader(TEST_FILE_BAD_TXT);
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileWriter()
   {
      File testFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH.toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         PrintWriter writer = DeprecatedFileTools.getFileWriter(testFile1.getAbsolutePath());
         assertNotNull(writer);
         writer.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   
      testFile1.delete();
      createTestFile1();
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileWriterWithAppend()
   {
      File testFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH.toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         PrintWriter writer = DeprecatedFileTools.getFileWriterWithAppend(testFile1.getAbsolutePath());
         assertNotNull(writer);
         writer.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   
      testFile1.delete();
      createTestFile1();
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileDataOutputStream()
   {
      File testFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH.toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         DataOutputStream outStream = DeprecatedFileTools.getFileDataOutputStream(testFile1.getAbsolutePath());
         assertNotNull(outStream);
         outStream.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   
      testFile1.delete();
      createTestFile1();
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileDataInputStream()
   {
      File testFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH.toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         DataInputStream inStream = DeprecatedFileTools.getFileDataInputStream(testFile1.getAbsolutePath());
         assertNotNull(inStream);
         inStream.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         fail();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = FileNotFoundException.class)
   public void testGetFileDataInputStreamWithFileNotFoundException() throws FileNotFoundException, IOException
   {
      DataInputStream inStream = DeprecatedFileTools.getFileDataInputStream(TEST_FILE_BAD_TXT);
      inStream.close();
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllFilesInDirectoryRecursive()
   {
      File directory = FILE_TOOLS_TEST_PATH.toFile();
      ArrayList<File> listOfFiles = DeprecatedFileTools.getAllFilesInDirectoryRecursive(directory);
      ArrayList<String> listOfFileNames = new ArrayList<String>();
      String errorMessage = "listOfFileNames that were found: ";
      for (File f : listOfFiles)
      {
         listOfFileNames.add(f.getName());
         errorMessage = errorMessage + f.getName() + " ";
      }
   
      assertTrue(listOfFileNames.contains(FILE_TOOLS_EXAMPLE_FILE1_TXT));
      assertTrue(listOfFileNames.contains(FILE_TOOLS_EXAMPLE_FILE2_TXT));
      assertTrue(listOfFileNames.contains(EXAMPLE_JAVA_FILE1_JAVA));
      assertTrue(listOfFileNames.contains(EXAMPLE_JAVA_FILE2_JAVA));
   
      boolean thrown = false;
      File notADirectoryExceptionTestFile = EXAMPLE_JAVA_FILE1_PATH.toFile();
   
      try
      {
         DeprecatedFileTools.getAllFilesInDirectoryRecursive(notADirectoryExceptionTestFile);
      }
      catch (RuntimeException e)
      {
         thrown = true;
      }
   
      assertTrue(thrown);
   }

   @SuppressWarnings("deprecation")
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllFilesInDirectoryWithSuffix()
   {
      File textFileDirectory = TEXT_DIRECTORY_PATH.toFile();
      File notADirectoryExceptionTestFile = EXAMPLE_JAVA_FILE1_PATH.toFile();
      File javaFileDirectory = JAVA_DIRECTORY_PATH.toFile();
      ArrayList<File> textFiles = DeprecatedFileTools.getAllFilesInDirectoryWithSuffix("txt", textFileDirectory);
      ArrayList<File> javaFiles = DeprecatedFileTools.getAllFilesInDirectoryWithSuffix("java.txt", javaFileDirectory);
   
      ArrayList<String> listOfTextFileNames = new ArrayList<String>();
      String errorMessage = "listOfTextFileNames that were found: ";
      for (File f : textFiles)
      {
         listOfTextFileNames.add(f.getName());
         errorMessage = errorMessage + f.getName() + " ";
      }
   
      assertEquals(errorMessage, 2, textFiles.size());
   
      ArrayList<String> listOfJavaFileNames = new ArrayList<String>();
      errorMessage = "listOfJavaFileNames that were found: ";
   
      for (File f : javaFiles)
      {
         listOfJavaFileNames.add(f.getName());
         errorMessage = errorMessage + f.getName();
      }
   
      assertEquals(errorMessage, 2, javaFiles.size());
   
   
      assertTrue(listOfTextFileNames.contains(FILE_TOOLS_EXAMPLE_FILE1_TXT));
      assertTrue(listOfTextFileNames.contains(FILE_TOOLS_EXAMPLE_FILE2_TXT));
      assertTrue(listOfJavaFileNames.contains(EXAMPLE_JAVA_FILE1_JAVA));
      assertTrue(listOfJavaFileNames.contains(EXAMPLE_JAVA_FILE2_JAVA));
   
      // test non directory search
      boolean thrown = false;
      try
      {
         DeprecatedFileTools.getAllFilesInDirectoryWithSuffix("txt", notADirectoryExceptionTestFile);
      }
      catch (RuntimeException e)
      {
         thrown = true;
      }
   
      assertTrue(thrown);
   
      // test searching empty directory
      File emptyDirectory = EMPTY_DIRECTORY_PATH.toFile();
   
      ArrayList<File> testEmptyDir = DeprecatedFileTools.getAllFilesInDirectoryWithSuffix("*", emptyDirectory);
      assertEquals(0, testEmptyDir.size());
   }
}
