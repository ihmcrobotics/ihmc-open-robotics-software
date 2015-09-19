package us.ihmc.tools.io.files;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
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
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.io.files.FileTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class FileToolsTest
{
   private static final Path FILE_TOOLS_TEST_PATH = JUnitTools.deriveTestResourcesPath(FileToolsTest.class);
   private static final Path TEXT_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleTextFiles");
   private static final Path JAVA_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleJavaFiles");
   private static final Path EMPTY_DIRECTORY_PATH = FILE_TOOLS_TEST_PATH.resolve("exampleEmptyFiles");
   
   private static final String EXAMPLE_FILE_1_TEXT_LINE_1 = "This is example File 1 !!&&#))(";
   private static final String EXAMPLE_FILE_2_TEXT_LINE_1 = "This is example File 2 *@&&%*@";
   private static final String EXAMPLE_FILE_2_TEXT_LINE_2 = "It has two lines";

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
   }
   
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testReadAllLines()
   {
      String exampleFile = "line1\nline2\nline3\n";
      
      Path testReadAllLinesPath = FILE_TOOLS_TEST_PATH.resolve("testReadAllLines.txt");
      
      PrintWriter writer = FileTools.newPrintWriter(testReadAllLinesPath);
      writer.print(exampleFile);
      writer.close();
      
      List<String> lines = FileTools.readAllLines(testReadAllLinesPath);
      
      assertTrue(lines.get(0).equals("line1"));
      assertTrue(lines.get(1).equals("line2"));
      assertTrue(lines.get(2).equals("line3"));
   }
	
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTemporaryDirectoryPath()
   {
      String tempPath = FileTools.getTemporaryDirectoryPath().toString();
      PrintTools.info(this, "Java temp directory: " + tempPath);
      assertNotNull("Java temp directory is null.", tempPath);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConcatenateFilesTogether()
   {      
      Path concatFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt");
      Path concatFile2 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile2.txt");
      Path concatedFile = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFileCat.txt");

      List<Path> filesToConcat = new ArrayList<Path>();
      filesToConcat.add(concatFile1);
      filesToConcat.add(concatFile2);

      FileTools.concatenateFilesTogether(filesToConcat, concatedFile);

      try
      {
         BufferedReader reader = FileTools.newBufferedReader(concatedFile);
         assertEquals(EXAMPLE_FILE_1_TEXT_LINE_1, reader.readLine());
         assertEquals(EXAMPLE_FILE_2_TEXT_LINE_1, reader.readLine());
         assertEquals(EXAMPLE_FILE_2_TEXT_LINE_2, reader.readLine());
         assertNull(reader.readLine());
         reader.close();
         
         Files.delete(concatedFile);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

   private static void createTestFile1()
   {
      PrintWriter writer = FileTools.newPrintWriter(TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt"));
      writer.println(EXAMPLE_FILE_1_TEXT_LINE_1);
      writer.flush();
      writer.close();
   }

   private static void createTestFile2()
   {
      PrintWriter writer = FileTools.newPrintWriter(TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile2.txt"));
      writer.println(EXAMPLE_FILE_2_TEXT_LINE_1);
      writer.println(EXAMPLE_FILE_2_TEXT_LINE_2);
      writer.flush();
      writer.close();
   }

   private static void createJavaFile1()
   {
      PrintWriter writer = FileTools.newPrintWriter(JAVA_DIRECTORY_PATH.resolve("ExampleJavaFile1.java"));
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
      PrintWriter writer = FileTools.newPrintWriter(JAVA_DIRECTORY_PATH.resolve("ExampleJavaFile2.java"));
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

   // START DEPRECATED TESTS HERE
   
	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetBufferedReader()
   {
      File testFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt").toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         BufferedReader reader = FileTools.getFileReader(testFile1.getAbsolutePath());
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

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = FileNotFoundException.class)
   public void testGetBufferedReaderWithFileNotFoundException() throws FileNotFoundException
   {
      BufferedReader reader = FileTools.getFileReader("testFileBad.txt");
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileWriter()
   {
      File testFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt").toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         PrintWriter writer = FileTools.getFileWriter(testFile1.getAbsolutePath());
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

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileWriterWithAppend()
   {
      File testFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt").toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         PrintWriter writer = FileTools.getFileWriterWithAppend(testFile1.getAbsolutePath());
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

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileDataOutputStream()
   {
      File testFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt").toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         DataOutputStream outStream = FileTools.getFileDataOutputStream(testFile1.getAbsolutePath());
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

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFileDataInputStream()
   {
      File testFile1 = TEXT_DIRECTORY_PATH.resolve("FileToolsExampleFile1.txt").toFile();
      if (!testFile1.exists())
         createTestFile1();
   
      try
      {
         DataInputStream inStream = FileTools.getFileDataInputStream(testFile1.getAbsolutePath());
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

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = FileNotFoundException.class)
   public void testGetFileDataInputStreamWithFileNotFoundException() throws FileNotFoundException, IOException
   {
      DataInputStream inStream = FileTools.getFileDataInputStream("testFileBad.txt");
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllFilesInDirectoryRecursive()
   {
      File directory = FILE_TOOLS_TEST_PATH.toFile();
      ArrayList<File> listOfFiles = FileTools.getAllFilesInDirectoryRecursive(directory);
      ArrayList<String> listOfFileNames = new ArrayList<String>();
      String errorMessage = "listOfFileNames that were found: ";
      for (File f : listOfFiles)
      {
         listOfFileNames.add(f.getName());
         errorMessage = errorMessage + f.getName() + " ";
      }
   
      assertTrue(listOfFileNames.contains("FileToolsExampleFile1.txt"));
      assertTrue(listOfFileNames.contains("FileToolsExampleFile2.txt"));
      assertTrue(listOfFileNames.contains("ExampleJavaFile1.java"));
      assertTrue(listOfFileNames.contains("ExampleJavaFile2.java"));
   
      boolean thrown = false;
      File notADirectoryExceptionTestFile = JAVA_DIRECTORY_PATH.resolve("ExampleJavaFile1.java").toFile();
   
      try
      {
         FileTools.getAllFilesInDirectoryRecursive(notADirectoryExceptionTestFile);
      }
      catch (RuntimeException e)
      {
         thrown = true;
      }
   
      assertTrue(thrown);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllFilesInDirectoryWithSuffix()
   {
      File textFileDirectory = TEXT_DIRECTORY_PATH.toFile();
      File notADirectoryExceptionTestFile = JAVA_DIRECTORY_PATH.resolve("ExampleJavaFile1.java").toFile();
      File javaFileDirectory = JAVA_DIRECTORY_PATH.toFile();
      ArrayList<File> textFiles = FileTools.getAllFilesInDirectoryWithSuffix("txt", textFileDirectory);
      ArrayList<File> javaFiles = FileTools.getAllFilesInDirectoryWithSuffix("java", javaFileDirectory);
   
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
   
   
      assertTrue(listOfTextFileNames.contains("FileToolsExampleFile1.txt"));
      assertTrue(listOfTextFileNames.contains("FileToolsExampleFile2.txt"));
      assertTrue(listOfJavaFileNames.contains("ExampleJavaFile1.java"));
      assertTrue(listOfJavaFileNames.contains("ExampleJavaFile2.java"));
   
      // test non directory search
      boolean thrown = false;
      try
      {
         FileTools.getAllFilesInDirectoryWithSuffix("txt", notADirectoryExceptionTestFile);
      }
      catch (RuntimeException e)
      {
         thrown = true;
      }
   
      assertTrue(thrown);
   
      // test empty directory search
      ArrayList<File> testFile = FileTools.getAllFilesInDirectoryWithSuffix("txt", new File(""));
   
      // test searching empty directory
      File textDirectory = TEXT_DIRECTORY_PATH.toFile();
      File emptyDirectory = EMPTY_DIRECTORY_PATH.toFile();
   
      ArrayList<File> testEmptyDir = FileTools.getAllFilesInDirectoryWithSuffix("*", emptyDirectory);
      assertEquals(0, testEmptyDir.size());
   }
}
