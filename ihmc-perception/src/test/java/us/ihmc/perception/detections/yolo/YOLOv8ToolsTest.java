package us.ihmc.perception.detections.yolo;

import static org.junit.jupiter.api.Assertions.*;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

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
      if (Files.exists(testDirectoryPath))
         deleteRecursively(testDirectoryPath);

      Files.createDirectories(goodYoloModelDirectory.toPath());
      Files.createDirectories(badYoloModelDirectory.toPath());
      Files.createFile(validONNXFile.toPath());
      Files.createFile(validClassNameFile.toPath());
      Files.createFile(someOtherFile.toPath());
      Files.createFile(randomFile.toPath());

      testDirectory.deleteOnExit();
   }

   private static void deleteRecursively(Path root) throws IOException
   {
      try (Stream<Path> paths = Files.walk(root))
      {
         paths.sorted(Comparator.reverseOrder()).forEach(path ->
         {
            try
            {
               Files.delete(path);
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         });
      }
   }

   @Test
   public void testIsValidYOLOModelDirectory()
   {
      assertTrue(YOLOv8Tools.isValidYOLOModelDirectory(goodYoloModelDirectory.toPath()));
      assertFalse(YOLOv8Tools.isValidYOLOModelDirectory(badYoloModelDirectory.toPath()));
   }

   @Test
   public void testGetYoloModelDirectories() throws MalformedURLException
   {
      List<URL> yoloModelDirectories = YOLOv8Tools.getYOLOModelDirectories(testDirectoryPath.toUri().toURL());
      assertEquals(1, yoloModelDirectories.size());

      assertTrue(yoloModelDirectories.contains(goodYoloModelDirectory.toURI().toURL()));
      assertFalse(yoloModelDirectories.contains(badYoloModelDirectory.toURI().toURL()));
   }

   @Test
   public void testGetFiles() throws MalformedURLException
   {
      // Getting ONNX file
      assertEquals(validONNXFile.toURI().toURL(), YOLOv8Tools.getONNXFile(goodYoloModelDirectory.toURI().toURL()));
      assertThrows(IllegalArgumentException.class, () -> YOLOv8Tools.getONNXFile(badYoloModelDirectory.toURI().toURL()));

      // Getting class names file
      assertEquals(validClassNameFile.toURI().toURL(), YOLOv8Tools.getClassNamesFile(goodYoloModelDirectory.toURI().toURL()));
      assertThrows(IllegalArgumentException.class, () -> YOLOv8Tools.getClassNamesFile(badYoloModelDirectory.toURI().toURL()));
   }

   @Test
   public void testResizeWithCropDownscaleCenterCrop()
   {
      Mat input = new Mat(100, 200, opencv_core.CV_8UC1);
      Mat output = new Mat();

      for (int y = 0; y < input.rows(); y++)
      {
         for (int x = 0; x < input.cols(); x++)
            input.ptr(y, x).put((byte) x);
      }

      YOLOv8Tools.resizeWithCrop(input, output, new Size(100, 100));

      assertEquals(100, output.cols());
      assertEquals(100, output.rows());
      assertEquals(50, getUnsignedByte(output, 50, 0));
      assertEquals(100, getUnsignedByte(output, 50, 50));
      assertEquals(149, getUnsignedByte(output, 50, 99));

      input.close();
      output.close();
   }

   @Test
   public void testResizeWithCropUpscaleCenterCrop()
   {
      Mat input = new Mat(100, 50, opencv_core.CV_8UC1);
      Mat output = new Mat();

      for (int y = 0; y < input.rows(); y++)
      {
         for (int x = 0; x < input.cols(); x++)
            input.ptr(y, x).put((byte) y);
      }

      YOLOv8Tools.resizeWithCrop(input, output, new Size(200, 200));

      assertEquals(200, output.cols());
      assertEquals(200, output.rows());

      int topSample = getUnsignedByte(output, 0, 100);
      int centerSample = getUnsignedByte(output, 100, 100);
      int bottomSample = getUnsignedByte(output, 199, 100);

      assertTrue(topSample < centerSample);
      assertTrue(centerSample < bottomSample);
      assertTrue(centerSample >= 49 && centerSample <= 51);

      input.close();
      output.close();
   }

   private static int getUnsignedByte(Mat mat, int row, int col)
   {
      return mat.ptr(row, col).get() & 0xFF;
   }
}
