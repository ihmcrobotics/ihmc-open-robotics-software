package us.ihmc.perception.filters;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.sensors.CameraIntrinsics;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class DepthImageFlyingPointsFilterTest
{
   private CameraIntrinsics cameraIntrinsics;
   private DepthImageFlyingPointsFilter flyingPointsFilter;

   @BeforeEach
   public void setupVariables()
   {
      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      cameraIntrinsics = new CameraIntrinsics();
      flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
   }

   @AfterEach
   public void shutdownTest()
   {
      flyingPointsFilter.destroy();
   }

   /**
    * This test makes sure that we don't detect flying points on a section that doesn't have any problems
    */
   @Test
   public void testFlyingPointsDontGetDetected_0()
   {
      // Don't format this grid because it's easier to visualize this way
      short[] depthValues = new short[]
            {
                  1940, 1938, 1936, 1934, 1932, 1932, 1930,
                  1940, 1938, 1936, 1934, 1932, 1932, 1930,
                  1938, 1936, 1934, 1932, 1930, 1929, 1927,
                  1936, 1934, 1932, 1930, 1929, 1927, 1925,
                  1936, 1932, 1930, 1929, 1927, 1925, 1923,
                  1934, 1932, 1930, 1927, 1925, 1923, 1921,
            };

      // u is for the columns, v is for the rows (7 x 6)
      int u = 7;
      int v = 6;
      setupCameraIntrinsics(u, v);

      // Setup mat to pass into the filter
      Mat depthMat = new Mat(u, v, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);

      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      checkResultFromFilter(outputFilteredDepthImage, depthMat);
   }

   /**
    * This test makes sure that we don't detect flying points on a section that doesn't have any problems
    */
   @Test
   public void testFlyingPointsDontGetDetected_1()
   {
      // Don't format this grid because it's easier to visualize this way
      short[] depthValues = new short[]
         {
            1953, 1953, 1955, 1955, 1955, 1953, 1953, 1951, 1949, 1948, 1946, 1944, 1942, 1940, 1938, 1936, 1934, 1932, 1932, 1930, 1929, 1929, 1927, 1927, 1927,
            1953, 1953, 1955, 1955, 1955, 1953, 1953, 1951, 1949, 1948, 1946, 1944, 1942, 1940, 1938, 1936, 1934, 1932, 1932, 1930, 1929, 1929, 1927, 1927, 1927,
            1955, 1955, 1955, 1955, 1955, 1953, 1953, 1951, 1949, 1948, 1946, 1944, 1940, 1938, 1936, 1934, 1932, 1930, 1929, 1927, 1927, 1927, 1925, 1925, 1923,
            1955, 1955, 1955, 1955, 1955, 1953, 1953, 1951, 1949, 1948, 1944, 1942, 1940, 1936, 1934, 1932, 1930, 1929, 1927, 1925, 1923, 1923, 1921, 1921, 1921,
            1953, 1955, 1955, 1955, 1955, 1955, 1953, 1951, 1949, 1948, 1944, 1942, 1938, 1936, 1932, 1930, 1929, 1927, 1925, 1923, 1921, 1919, 1919, 1917, 1917,
            1953, 1953, 1953, 1955, 1955, 1953, 1953, 1949, 1948, 1946, 1944, 1940, 1938, 1934, 1932, 1930, 1927, 1925, 1923, 1921, 1919, 1917, 1915, 1915, 1915,
            1951, 1953, 1953, 1953, 1953, 1953, 1951, 1949, 1948, 1946, 1942, 1940, 1938, 1934, 1932, 1930, 1927, 1925, 1923, 1921, 1919, 1917, 1915, 1914, 1914,
            1951, 1951, 1951, 1953, 1951, 1951, 1949, 1948, 1946, 1944, 1942, 1938, 1936, 1934, 1932, 1930, 1927, 1925, 1923, 1921, 1919, 1917, 1915, 1914, 1914,
            1948, 1949, 1949, 1949, 1949, 1949, 1948, 1946, 1944, 1942, 1940, 1938, 1936, 1934, 1932, 1930, 1929, 1927, 1925, 1923, 1921, 1919, 1917, 1915, 1914,
            1946, 1948, 1948, 1948, 1946, 1946, 1944, 1944, 1942, 1940, 1938, 1936, 1934, 1932, 1930, 1930, 1929, 1927, 1925, 1925, 1921, 1919, 1919, 1917, 1915,
            1942, 1944, 1944, 1944, 1944, 1942, 1942, 1940, 1938, 1938, 1936, 1934, 1932, 1932, 1930, 1930, 1929, 1927, 1925, 1925, 1923, 1921, 1919, 1917, 1917,
            1938, 1940, 1940, 1940, 1940, 1938, 1938, 1938, 1936, 1934, 1932, 1932, 1932, 1930, 1930, 1929, 1927, 1927, 1925, 1925, 1925, 1923, 1921, 1919, 1917,
            1936, 1936, 1936, 1936, 1936, 1934, 1934, 1934, 1932, 1932, 1930, 1930, 1930, 1929, 1929, 1927, 1927, 1927, 1925, 1925, 1925, 1923, 1921, 1919, 1917,
            1932, 1932, 1932, 1932, 1932, 1932, 1932, 1930, 1930, 1929, 1929, 1927, 1927, 1927, 1927, 1927, 1927, 1927, 1925, 1925, 1925, 1923, 1923, 1921, 1919,
            1930, 1930, 1930, 1929, 1929, 1929, 1929, 1927, 1927, 1927, 1927, 1925, 1925, 1925, 1925, 1925, 1925, 1925, 1925, 1925, 1925, 1923, 1923, 1921, 1919,
            1927, 1927, 1927, 1927, 1927, 1925, 1925, 1925, 1925, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1923, 1921, 1921, 1919, 1919,
            1925, 1925, 1925, 1923, 1923, 1923, 1923, 1923, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1921, 1919, 1919, 1917,
            1923, 1923, 1923, 1923, 1923, 1921, 1921, 1921, 1921, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1919, 1917, 1917, 1917, 1915,
            1923, 1921, 1921, 1921, 1919, 1919, 1919, 1919, 1919, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1917, 1915, 1915, 1915, 1914,
            1921, 1921, 1919, 1919, 1919, 1919, 1917, 1917, 1917, 1917, 1915, 1915, 1915, 1915, 1915, 1915, 1915, 1915, 1915, 1915, 1915, 1914, 1914, 1914, 1912,
            1919, 1919, 1919, 1917, 1917, 1917, 1917, 1917, 1915, 1915, 1915, 1915, 1914, 1914, 1914, 1915, 1914, 1914, 1914, 1914, 1914, 1914, 1912, 1912, 1910,
            1917, 1917, 1917, 1917, 1917, 1917, 1915, 1915, 1915, 1914, 1914, 1914, 1914, 1914, 1914, 1914, 1914, 1914, 1914, 1914, 1912, 1912, 1912, 1910, 1910,
            1917, 1917, 1917, 1915, 1915, 1915, 1915, 1914, 1914, 1914, 1914, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1910, 1910, 1910, 1908,
            1917, 1917, 1915, 1915, 1915, 1915, 1914, 1914, 1914, 1914, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1912, 1910, 1910, 1910, 1910, 1908, 1908, 1906,
            1915, 1915, 1915, 1915, 1914, 1914, 1914, 1914, 1914, 1912, 1912, 1912, 1912, 1912, 1910, 1910, 1910, 1910, 1910, 1910, 1910, 1908, 1908, 1906, 1906
         };

      int u = 25;
      int v = 25;
      Mat depthMat = new Mat(u, v, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);

      setupCameraIntrinsics(u, v);

      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      checkResultFromFilter(outputFilteredDepthImage, depthMat);
   }

   /**
    * This test makes sure that we don't detect flying points on a section that doesn't have any problems
    */
   @Test
   public void testFlyingPointsDontGetDetected_2()
   {
      // Don't format this grid because it's easier to visualize this way
      short[] depthValues = new short[]
         {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 502, 502, 502, 502, 501, 501, 501, 501, 500, 0, 500, 500, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 502, 502, 502, 502, 501, 501, 501, 501, 501, 501, 500, 500,
            0, 0, 0, 0, 0, 0, 0, 0, 502, 502, 502, 502, 502, 501, 501, 501, 501, 501, 501, 501,
            0, 0, 0, 0, 0, 0, 0, 0, 502, 502, 502, 502, 502, 501, 501, 501, 501, 501, 501, 501,
            0, 0, 0, 0, 0, 0, 0, 0, 502, 502, 502, 502, 502, 501, 501, 501, 501, 501, 501, 501,
            0, 0, 0, 0, 0, 502, 502, 502, 502, 502, 501, 501, 0, 501, 501, 501, 501, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 502, 502, 502, 502, 501, 501, 501, 501, 501, 500, 500, 500, 500,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 502, 502, 501, 501, 501, 501, 500, 500, 500, 499,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 502, 501, 501, 501, 500, 500, 500,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
         };

      int u = 20;
      int v = 20;

      Mat depthMat = new Mat(u, v, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      setupCameraIntrinsics(u, v);
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      checkResultFromFilter(outputFilteredDepthImage, depthMat);
   }

   @Test
   public void testFlyingPoints() throws IOException, URISyntaxException
   {
      Path path = Paths.get(Objects.requireNonNull(DepthImageFlyingPointsFilterTest.class.getResource("depthImageAsBytes.raw")).toURI());
      byte[] imageBytes = Files.readAllBytes(path);
      Mat depthMat = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(imageBytes));

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setCx(640.0);
      cameraIntrinsics.setCy(360.0);
      cameraIntrinsics.setFx(720);
      cameraIntrinsics.setFy(1280);

      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      boolean somethingChanged = false;

      for (int i = 0; i < result.rows(); i++)
      {
         for (int j = 0; j < result.cols(); j++)
         {
            if (depthMat.row(i).col(j).data().getShort() != result.row(i).col(j).data().getShort())
            {
               somethingChanged = true;
            }
         }
      }

         assertTrue(somethingChanged);

   }

   private static void checkResultFromFilter(GpuMat outputFilteredDepthImage, Mat expectedValue)
   {
      Mat actualValue = new Mat();
      outputFilteredDepthImage.download(actualValue);

      for (int i = 0; i < actualValue.rows(); i++)
      {
         for (int j = 0; j < actualValue.cols(); j++)
         {
            assertEquals(expectedValue.row(i).col(j).data().getShort(),
                         actualValue.row(i).col(j).data().getShort(),
                         "The filtered data does not match the expected data!" + "Input Data: " + expectedValue.row(i).col(j).data().getShort()
                         + "\n Filtered Data: " + actualValue.row(i).col(j).data().getShort() + ".\n" + "Row: " + i + " and Column: " + j + "!");
         }
      }
   }

   /**
    * These are the camera intrinsics.
    * The values hardcoded are pulled from the RealSense camera
    * We scale those values by the current grid size to get an accurate representation for our smaller grid.
    */
   private void setupCameraIntrinsics(int uSize, int vSize)
   {
      cameraIntrinsics.setCx(uSize / 640.0);
      cameraIntrinsics.setCy(vSize / 360.0);
      cameraIntrinsics.setFx((double) uSize / 720);
      cameraIntrinsics.setFy((double) vSize / 1280);
   }
}
