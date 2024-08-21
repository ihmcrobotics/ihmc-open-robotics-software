package us.ihmc.perception.detections.yolo;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class YOLOv8ToolsTest
{
   private static final String GOOD_MODEL_DIRECTORY_NAME = "yoloModelA";
   private static final String BAD_MODEL_DIRECTORY_NAME = "yoloModelB";
   private static final String ONNX_FILE_NAME = "yolo.onnx";

   private static final Path testDirectoryPath = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("yolo-tools-test");
   private static final File testDirectory = testDirectoryPath.toFile();
   private static final File goodYoloModelDirectory = new File(testDirectory, GOOD_MODEL_DIRECTORY_NAME);
   private static final File validONNXFile = new File(goodYoloModelDirectory, ONNX_FILE_NAME);
   private static final File validClassNameFile = new File(goodYoloModelDirectory, YOLOv8Tools.CLASS_NAME_FILE_NAME);
   private static final File someOtherFile = new File(goodYoloModelDirectory, "Hello.txt");
   private static final File badYoloModelDirectory = new File(testDirectory, BAD_MODEL_DIRECTORY_NAME);
   private static final File randomFile = new File(badYoloModelDirectory, "Hello.txt");


   @BeforeAll
   public static void createTestingDirectory() throws IOException
   {
      // create temporary testing directory
      assertTrue(testDirectory.mkdir());
      testDirectory.deleteOnExit();

      // create good model directory
      assertTrue(goodYoloModelDirectory.mkdir());
      goodYoloModelDirectory.deleteOnExit();

      assertTrue(validONNXFile.createNewFile());
      validONNXFile.deleteOnExit();

      assertTrue(validClassNameFile.createNewFile());
      validClassNameFile.deleteOnExit();

      assertTrue(someOtherFile.createNewFile());
      someOtherFile.deleteOnExit();

      // create bad model directory
      assertTrue(badYoloModelDirectory.mkdir());
      badYoloModelDirectory.deleteOnExit();

      assertTrue(randomFile.createNewFile());
      randomFile.deleteOnExit();
   }

   @Test
   public void testIsValidYOLOModelDirectory()
   {
      assertTrue(YOLOv8Tools.isValidYOLOModelDirectory(goodYoloModelDirectory.toPath()));
      assertFalse(YOLOv8Tools.isValidYOLOModelDirectory(badYoloModelDirectory.toPath()));
   }

   @Test
   public void testGetYoloModelDirectories()
   {
      List<Path> yoloModelDirectories = YOLOv8Tools.getYOLOModelDirectories(testDirectoryPath);
      assertEquals(1, yoloModelDirectories.size());

      assertTrue(yoloModelDirectories.contains(goodYoloModelDirectory.toPath()));
      assertFalse(yoloModelDirectories.contains(badYoloModelDirectory.toPath()));
   }

   @Test
   public void testGetFiles()
   {
      // Getting ONNX file
      assertEquals(validONNXFile.toPath(), YOLOv8Tools.getONNXFile(goodYoloModelDirectory.toPath()));
      assertThrows(IllegalArgumentException.class, () -> YOLOv8Tools.getONNXFile(badYoloModelDirectory.toPath()));

      // Getting class names file
      assertEquals(validClassNameFile.toPath(), YOLOv8Tools.getClassNamesFile(goodYoloModelDirectory.toPath()));
      assertThrows(IllegalArgumentException.class, () -> YOLOv8Tools.getClassNamesFile(badYoloModelDirectory.toPath()));
   }
}
