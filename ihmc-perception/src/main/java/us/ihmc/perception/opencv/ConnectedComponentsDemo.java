package us.ihmc.perception.opencv;

import gnu.trove.list.array.TDoubleArrayList;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.tools.IHMCCommonPaths;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.bytedeco.opencv.global.opencv_core.CV_8U;
import static org.bytedeco.opencv.global.opencv_core.minMaxLoc;

public class ConnectedComponentsDemo
{
   public static void main(String[] args) throws IOException
   {
      Path heightMapDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY;
//      Path path = heightMapDirectory.resolve("20250908_124733_082_HeightMapLog.json");
      Path path = heightMapDirectory.resolve("20250908_150921_664_HeightMapLog.json");

      JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
      HeightMapMessage heightMapMessage = serializer.deserialize(path.toFile());
      Mat heights = unpackMessageToMat(heightMapMessage);

      float min = heightMapMessage.getHeights().min();
      float max = heightMapMessage.getHeights().max();
      BufferedImage heightImage = heightMapToBufferedImage(heights, min, max);

      displayImage(heightImage);

      Mat steppability = computeSteppability(heights, heightMapMessage.getCellSizeInMeters());
      BufferedImage steppabilityImage = steppabilityToBufferedImage(steppability);

      displayImage(steppabilityImage);

      //      GpuMat steppability = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32S);
//      GpuMat steppabilityThresholded = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32S);
//      opencv_cudaarithm.threshold(steppability, steppabilityThresholded, 0.2, 255.0, opencv_imgproc.THRESH_BINARY);

      int cellsPerAxis = heightMapMessage.getCellsPerAxis();
      Mat labels = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32S);
      GpuMat deviceInput = new GpuMat();
      GpuMat deviceLabels = new GpuMat();
      deviceInput.upload(steppability);

      opencv_cudaimgproc.connectedComponents(deviceInput, deviceLabels);

      deviceLabels.download(labels);
      BufferedImage bufferedImage = labelsToBufferedImage(labels);

      displayImage(bufferedImage);

//      BufferedImage labelsImage = labelsToBufferedImage(labels);
   }

   private static void displayImage(BufferedImage bufferedImage)
   {
      JFrame frame = new JFrame();
      frame.getContentPane().setLayout(new FlowLayout());
      frame.getContentPane().add(new JLabel(new ImageIcon(bufferedImage)));
      frame.pack();
      frame.setVisible(true);
   }

   private static Mat unpackMessageToMat(HeightMapMessage message)
   {
      int cellsPerAxis = message.getCellsPerAxis();
      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      FloatBuffer floatBuffer = heightMap.createBuffer();

      int totalCells = cellsPerAxis * cellsPerAxis;
      float[] heights = new float[totalCells];

      for (int key = 0; key < message.getHeights().size(); key++)
      {
         heights[key] = message.getHeights().get(key);
      }

      floatBuffer.put(heights);
      return heightMap;
   }

   private static BufferedImage heightMapToBufferedImage(Mat mat, float min, float max)
   {
      FloatBuffer buffer = mat.createBuffer();

      int type = BufferedImage.TYPE_BYTE_GRAY;
      BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
      byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();

      for (int ix = 0; ix < mat.rows(); ix++)
      {
         for (int iy = 0; iy < mat.cols(); iy++)
         {
            int key = ix + mat.cols() * iy;
            double alphaHeight = EuclidCoreTools.clamp((buffer.get(key) - min) / (max - min), 0.0, 1.0);
            targetPixels[key] = (byte) (255 * alphaHeight);
         }
      }

      return image;
   }

   private static BufferedImage steppabilityToBufferedImage(Mat mat)
   {
      int type = BufferedImage.TYPE_BYTE_GRAY;
      BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
      byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();

      ByteBuffer buffer = mat.createBuffer();
      buffer.get(targetPixels);

      return image;
   }

   private static BufferedImage labelsToBufferedImage(Mat labels)
   {
      // Find maximum label value
      DoublePointer minVal = new DoublePointer(1);
      DoublePointer maxVal = new DoublePointer(1);
      minMaxLoc(labels, minVal, maxVal, null, null, null);
      double maxLabel = maxVal.get();

      // Convert to 8U, scaling so max label maps to 255
      Mat labels8U = new Mat();
      labels.convertTo(labels8U, CV_8U, 255.0 / Math.max(1.0, maxLabel), 0.0);

      // Wrap into BufferedImage
      BufferedImage image = new BufferedImage(labels8U.cols(), labels8U.rows(), BufferedImage.TYPE_BYTE_GRAY);
      byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
      labels8U.data().get(targetPixels);

      return image;
   }

   private static Mat computeSteppability(Mat heights, double gridResolution)
   {
      byte[] steppabilityArray = new byte[heights.rows() * heights.cols()];
      FloatBuffer heightsBuffer = heights.createBuffer();

      LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

      TDoubleArrayList normals = new TDoubleArrayList();
      TDoubleArrayList squaredErrors = new TDoubleArrayList();

      int n = 6;
      for (int ix = n; ix < heights.rows() - n; ix++)
      {
         for (int iy = n; iy < heights.cols() - n; iy++)
         {
            // points to fit to plane
            List<Point3D> points = new ArrayList<>();

            for (int dx = -n; dx <= n; dx++)
            {
               for (int dy = -n; dy <= n; dy++)
               {
                  int key = ix + dx + heights.cols() * (iy + dy);
                  float height = heightsBuffer.get(key);
                  points.add(new Point3D(gridResolution * dx, gridResolution * dy, height));
               }
            }

            Plane3D plane = new Plane3D();
            double squaredError = planeFitter.fitPlaneToPoints(points, plane);

            // classify steppability
            boolean isSteppable = squaredError < 0.00005;
            steppabilityArray[ix + heights.cols() * iy] = (byte) (isSteppable ? 0xFF : 0x00);

            normals.add(plane.getNormalZ());
            squaredErrors.add(squaredError);
         }
      }

      Mat steppability = new Mat(heights.rows(), heights.cols(), opencv_core.CV_8UC1);
      ByteBuffer buffer = steppability.createBuffer();
      buffer.put(steppabilityArray);

      return steppability;
   }
}
