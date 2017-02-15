package us.ihmc.commons.nio;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.io.files.FileTools;
import us.ihmc.tools.io.printing.PrintTools;

public class FileToolsTest
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
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testReadAllLines()
   {
      List<String> lines = FileTools.readAllLines(READ_ALL_LINES_PATH);
      
      assertTrue(lines.get(0).equals("line1"));
      assertTrue(lines.get(1).equals("line2"));
      assertTrue(lines.get(2).equals("line3"));
   }
	
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testReadAllBytesAndReadLinesFromBytesAndReplaceLine()
   {
      byte[] bytes = FileTools.readAllBytes(READ_ALL_LINES_PATH);
      List<String> lines = FileTools.readLinesFromBytes(bytes);
      
      assertTrue(lines.get(0).equals("line1"));
      assertTrue(lines.get(1).equals("line2"));
      assertTrue(lines.get(2).equals("line3"));
      
      bytes = FileTools.replaceLineInFile(1, "line2Mod", bytes, lines);
      lines = FileTools.readLinesFromBytes(bytes);

      assertTrue(lines.get(0).equals("line1"));
      assertTrue(lines.get(1).equals("line2Mod"));
      assertTrue(lines.get(2).equals("line3"));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTemporaryDirectoryPath()
   {
      String tempPath = FileTools.getTemporaryDirectoryPath().toString();
      PrintTools.info(this, "Java temp directory: " + tempPath);
      assertNotNull("Java temp directory is null.", tempPath);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConcatenateFilesTogether()
   {      
      Path concatFile1 = FILE_TOOLS_EXAMPLE_FILE1_PATH;
      Path concatFile2 = FILE_TOOLS_EXAMPLE_FILE2_PATH;
      Path concatedFile = TEXT_DIRECTORY_PATH.resolve(FILE_TOOLS_EXAMPLE_FILE_CAT_TXT);

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
}
