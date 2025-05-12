package us.ihmc.perception.filters;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.camera.CameraIntrinsics;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class DepthImageFlyingPointsFilterTest
{
   @Test
   public void testFlyingPointsDontGetDetected_0()
   {

      short[] depthValues = new short[]
            {
                  1940, 1938, 1936, 1934, 1932, 1932, 1930,
                  1940, 1938, 1936, 1934, 1932, 1932, 1930,
                  1938, 1936, 1934, 1932, 1930, 1929, 1927,
                  1936, 1934, 1932, 1930, 1929, 1927, 1925,
                  1936, 1932, 1930, 1929, 1927, 1925, 1923,
                  1934, 1932, 1930, 1927, 1925, 1923, 1921,
      };


      Mat depthMat = new Mat(25, 25, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setCx(7/640.0);
      cameraIntrinsics.setCy(6/360.0);
      cameraIntrinsics.setFx(7/649.4571918977125); // Pixels per meter
      cameraIntrinsics.setFy(6/649.4571918977125); // Pixels per meter
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      for (int i = 0; i < result.rows(); i++)
      {
         for (int j = 0; j < result.cols(); j++)
         {
            assertEquals(depthMat.row(i).col(j).data().getShort(),
                         result.row(i).col(j).data().getShort(),
                         "The filtered data does not match the expected data!" + "Input Data: " + depthMat.row(i).col(j).data().getShort()
                         + "\n Filtered Data: " + result.row(i).col(j).data().getShort() + ".\n" + "Row: " + i + " and Column: " + j + "!");
         }
      }

      flyingPointsFilter.destroy();
   }      @Test
   public void testFlyingPointsDontGetDetected_1()
   {

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


      Mat depthMat = new Mat(25, 25, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setCx(25/640.0);
      cameraIntrinsics.setCy(25/360.0);
      cameraIntrinsics.setFx((double) 25 / 720); // Pixels per meter
      cameraIntrinsics.setFy((double) 25 / 1280); // Pixels per meter
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      for (int i = 0; i < result.rows(); i++)
      {
         for (int j = 0; j < result.cols(); j++)
         {
            assertEquals(depthMat.row(i).col(j).data().getShort(),
                         result.row(i).col(j).data().getShort(),
                         "The filtered data does not match the expected data!" + "Input Data: " + depthMat.row(i).col(j).data().getShort()
                         + "\n Filtered Data: " + result.row(i).col(j).data().getShort() + ".\n" + "Row: " + i + " and Column: " + j + "!");
         }
      }

      flyingPointsFilter.destroy();
   }

   @Test
   public void testFlyingPointsDontGetDetected_2()
   {

      short[] depthValues = new short[] {
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


      Mat depthMat = new Mat(20, 20, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      DepthImageFilteringParameters depthImageFilteringParameters = new DepthImageFilteringParameters();
      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(depthImageFilteringParameters);
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setCx(20/640.0);
      cameraIntrinsics.setCy(20/360.0);
      cameraIntrinsics.setFx((double) 20 / 720); // Pixels per meter
      cameraIntrinsics.setFy((double) 20 / 1280); // Pixels per meter
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      //      PerceptionDebugTools.printMat("Result", result, 1);

      for (int i = 0; i < result.rows(); i++)
      {
         for (int j = 0; j < result.cols(); j++)
         {
            assertEquals(depthMat.row(i).col(j).data().getShort(), result.row(i).col(j).data().getShort(), "The filtered data does not match the expected data!"
                                                                                                           + "Input Data: " + depthMat.row(i).col(j).data().getShort() +
                                                                                                           "\n Filtered Data: " + result.row(i).col(j).data().getShort());
         }
      }

      flyingPointsFilter.destroy();
   }

   @Test
   public void testFlyingPoints() throws IOException, URISyntaxException
   {

      Path path = Paths.get(Objects.requireNonNull(DepthImageFlyingPointsFilterTest.class.getResource("depthImageAsBytes.raw")).toURI());
      byte[] imageBytes = Files.readAllBytes(path);
      Mat depthMat = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(imageBytes));
      //      PerceptionDebugTools.printMat("s", depthMat, 1);

      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(new DepthImageFilteringParameters());
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setCx(640.0);
      cameraIntrinsics.setCy(360.0);
      cameraIntrinsics.setFx(720); // Pixels per meter
      cameraIntrinsics.setFy(1280); // Pixels per meter

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

      flyingPointsFilter.destroy();
   }
}
