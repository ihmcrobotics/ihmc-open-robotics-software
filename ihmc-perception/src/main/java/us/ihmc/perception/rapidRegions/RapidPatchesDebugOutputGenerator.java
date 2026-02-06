package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.nio.FloatBuffer;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class RapidPatchesDebugOutputGenerator
{
   private boolean enabled = false;
   private boolean showPointCloud = true;

   private Mat debugImage;
   private Scalar internalColor = new Scalar(0, 0, 255, 0);
   private Scalar boundaryColor = new Scalar(255, 255, 255, 0);
   private final RecyclingArrayList<Point3D32> debugPoints = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<UnitVector3D> debugNormals = new RecyclingArrayList<>(UnitVector3D::new);

   public void create(int height, int width)
   {
      debugImage = new Mat(height, width, opencv_core.CV_8UC4);
   }

   public void drawRegionInternalPatches(RapidPlanarRegion planarRegion, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      for (Point2D regionIndex : planarRegion.getRegionIndices())
      {
         int x = (int) regionIndex.getX();
         int y = (int) regionIndex.getY();
         int r = (planarRegion.getId() + 1) * 312 % 255;
         int g = (planarRegion.getId() + 1) * 123 % 255;
         int b = (planarRegion.getId() + 1) * 231 % 255;

         // Draw a filled rectangle for the patch instead of a single pixel
         Scalar color = new Scalar(r, g, b, 255);
         Point topLeft = new Point(x * patchWidth, y * patchHeight);
         Point bottomRight = new Point((x + 1) * patchWidth - 1, (y + 1) * patchHeight - 1);
         opencv_imgproc.rectangle(debugImage, topLeft, bottomRight, color, -1, opencv_imgproc.LINE_8, 0);
      }
   }

   public void drawRegionRing(RapidRegionRing regionRing, int patchSize)
   {
      if (!enabled)
         return;

      for (Vector2D boundaryIndex : regionRing.getBoundaryIndices())
      {
         int x = (int) boundaryIndex.getX();
         int y = (int) boundaryIndex.getY();
         int r = (regionRing.getIndex() + 1) * 130 % 255;
         int g = (regionRing.getIndex() + 1) * 227 % 255;
         int b = (regionRing.getIndex() + 1) * 332 % 255;

         // Draw a filled rectangle for the patch instead of a single pixel
         Scalar color = new Scalar(r, g, b, 255);
         Point topLeft = new Point(x * patchSize, y * patchSize);
         Point bottomRight = new Point((x + 1) * patchSize - 1, (y + 1) * patchSize - 1);
         opencv_imgproc.rectangle(debugImage, topLeft, bottomRight, color, -1, opencv_imgproc.LINE_8, 0);
      }
   }

   public void constructPointCloud(FloatBuffer buffer, int numberOfPoints, RigidBodyTransform transform)
   {
      if (!enabled)
         return;

      debugPoints.clear();
      for (int i = 0; i < numberOfPoints; i++)
      {
         float cx = buffer.get(i * 3);
         float cy = buffer.get(i * 3 + 1);
         float cz = buffer.get(i * 3 + 2);

         //LogTools.info("Point To Render: {} {} {}", cx, cy, cz);

         Point3D32 point = new Point3D32(cx, cy, cz);

         if (point.norm() > 0.1f)
         {
            point.applyTransform(transform);
            debugPoints.add().set(point);
            //LogTools.info("Point To Render: {} {} {}", cx, cy, cz);
         }
      }
   }

   public void drawInternalNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      drawNode(patchRow, patchCol, patchHeight, patchWidth, getColor(id));
   }

   public void drawBoundaryNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      drawNode(patchRow, patchCol, patchHeight, patchWidth, boundaryColor);
   }

   private void drawNode(int patchRow, int patchCol, int patchHeight, int patchWidth, Scalar color)
   {
      if (!enabled)
         return;

      opencv_imgproc.circle(debugImage, new Point(patchRow * patchHeight, patchCol * patchWidth), 2, color, -1, -1, 0);
   }

   private Scalar getColor(int id)
   {
      internalColor.red((id + 1) * 231 % 255);
      internalColor.green((id + 1) * 123 % 255);
      internalColor.blue((id + 1) * 312 % 255);
      return internalColor;
   }

   public void clearDebugImage()
   {
      debugImage.put(new Scalar(0, 0, 0, 0));
   }

   public void update(Mat inputDepthImage, FloatBuffer floatBuffer, RigidBodyTransform transform)
   {
      if (!enabled)
         return;

//      printPatchGraph(patchGraph);

      if(showPointCloud)
      {
         constructPointCloud(floatBuffer, inputDepthImage.rows() * inputDepthImage.cols(), transform);
      }
//      constructPointCloud(patchFeatureGrid.getCxImage(), patchFeatureGrid.getCyImage(), patchFeatureGrid.getCzImage());

      //      constructCentroidSurfelCloud(patchFeatureGrid.getCxImage(), patchFeatureGrid.getCyImage(), patchFeatureGrid.getCzImage(), patchFeatureGrid.getNxImage(),
//                                   patchFeatureGrid.getNyImage(), patchFeatureGrid.getNzImage());

      //PerceptionDebugTools.displayDepth("Depth", inputDepthImage, 1);
      //showDebugImage(1);
   }

   public void displayInputDepth(Mat depth, int delay)
   {
      if (!enabled)
         return;

      Mat depthDisplay = new Mat();
      OpenCVTools.clampTo8BitUnsignedChar(depth, depthDisplay, 0.0, 255.0);
      OpenCVTools.convert8BitGrayTo8BitRGBA(depthDisplay, depthDisplay);

      imshow("Depth", depthDisplay);
      int code = waitKeyEx(delay);
      if (code == 113)
      {
         System.exit(0);
      }
   }

   public void showDebugImage(int delay)
   {
      if (!enabled)
         return;

      PerceptionDebugTools.display("Debug Output", debugImage, delay);
   }

   public Mat getDebugImage()
   {
      return debugImage;
   }

   public void transformPoints(RigidBodyTransform transform)
   {
      if (!enabled)
         return;

      for(Point3D32 point : debugPoints)
      {
         point.applyTransform(transform);
      }
   }

   public RecyclingArrayList<Point3D32> getDebugPoints()
   {
      return debugPoints;
   }

   public RecyclingArrayList<UnitVector3D> getDebugNormals()
   {
      return debugNormals;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public void setShowPointCloud(boolean showPointCloud)
   {
      this.showPointCloud = showPointCloud;
   }
}
