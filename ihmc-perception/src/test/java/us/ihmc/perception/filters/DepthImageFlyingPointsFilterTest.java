package us.ihmc.perception.filters;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.sensorProcessing.filters.DepthImageFilterParameters;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

public class DepthImageFlyingPointsFilterTest
{

   @Test
   public void testFlyingPoints() throws IOException, URISyntaxException
   {
      Path path = Paths.get(Objects.requireNonNull(DepthImageFlyingPointsFilterTest.class.getResource("realsenseDepthImageAsBytes.raw")).toURI());
      byte[] imageBytes = Files.readAllBytes(path);

      GpuMat inputDepthImage = new GpuMat(720, 1280, opencv_core.CV_16UC1, new BytePointer(imageBytes));
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      DepthImageFlyingPointsFilter flyingPointsFilter = new DepthImageFlyingPointsFilter(new DepthImageFilterParameters());
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      flyingPointsFilter.applyFilter(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      PerceptionDebugTools.printMat("Result", result, 20);

      flyingPointsFilter.destroy();
   }
}
