package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import boofcv.struct.calib.IntrinsicParameters;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class LidarImageFusionRawData
{
   private final BufferedImage imageData;
   private final int imageWidth;
   private final int imageHeight;

   private final int[] labels;

   private final IntrinsicParameters intrinsicParameters;

   private final ArrayList<Segment> segmentedData = new ArrayList<Segment>();

   public LidarImageFusionRawData(Point3D[] pointCloud, BufferedImage bufferedImage, int[] labels, IntrinsicParameters intrinsic)
   {
      imageData = bufferedImage;
      imageWidth = bufferedImage.getWidth();
      imageHeight = bufferedImage.getHeight();

      this.labels = labels;

      intrinsicParameters = intrinsic;

      if (labels.length != imageWidth * imageHeight)
         throw new RuntimeException("newLabels length is different with size of image " + labels.length + ", (w)" + imageWidth + ", (h)" + imageHeight);

      int numberOfLabels = 0;
      for (int i = 0; i < imageWidth * imageHeight; i++)
         if (numberOfLabels == labels[i])
            numberOfLabels++;

      LogTools.info("numberOfLabels " + numberOfLabels);
      for (int i = 0; i < numberOfLabels; i++)
         segmentedData.add(new Segment(i));

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, intrinsicParameters);
         int arrayIndex = getArrayIndex(pixel[0], pixel[1]);
         int label = labels[arrayIndex];

         segmentedData.get(label).addPoint(new Point3D(point));
      }

      for (int i = 0; i < segmentedData.size(); i++)
      {
         LogTools.info("Segment " + i + " has " + segmentedData.get(i).points.size() + " points.");
      }
      initializeSegments();
   }

   private void initializeSegments()
   {
      for (int u = 1; u < imageWidth - 1; u++)
      {
         for (int v = 1; v < imageHeight - 1; v++)
         {
            int curLabel = labels[getArrayIndex(u, v)];
            int[] labelsOfAdjacentPixels = new int[4];
            labelsOfAdjacentPixels[0] = labels[getArrayIndex(u, v - 1)]; // N
            labelsOfAdjacentPixels[1] = labels[getArrayIndex(u, v + 1)]; // S
            labelsOfAdjacentPixels[2] = labels[getArrayIndex(u - 1, v)]; // W
            labelsOfAdjacentPixels[3] = labels[getArrayIndex(u + 1, v)]; // E

            for (int labelOfAdjacentPixel : labelsOfAdjacentPixels)
            {
               if (curLabel != labelOfAdjacentPixel)
               {
                  if (!segmentedData.get(curLabel).contains(labelOfAdjacentPixel))
                     segmentedData.get(curLabel).addAdjacentSegmentLabel(labelOfAdjacentPixel);
               }
            }
         }
      }

      for (int i = 0; i < segmentedData.size(); i++)
      {
         Segment segment = segmentedData.get(i);
         LogTools.info("Segment " + i + " adjacented by ");
         int[] adjacentLabels = segment.getAdjacentSegmentLabels();
         for (int j = 0; j < adjacentLabels.length; j++)
            LogTools.info("" + adjacentLabels[j]);

         segment.update();
      }
   }

   private int getArrayIndex(int u, int v)
   {
      return u + v * imageWidth;
   }

   public int numberOfLabels()
   {
      return segmentedData.size();
   }

   public double getLabelWeight(int label)
   {
      return segmentedData.get(label).getWeight();
   }

   public int[] getAdjacentLabels(TIntArrayList labels)
   {
      TIntArrayList uncompressedAdjacentLabels = new TIntArrayList();
      TIntArrayList adjacentLabels = new TIntArrayList();

      for (int label : labels.toArray())
      {
         uncompressedAdjacentLabels.addAll(segmentedData.get(label).getAdjacentSegmentLabels());
      }

      for (int i = 0; i < segmentedData.size(); i++)
      {
         if (uncompressedAdjacentLabels.contains(i) && !labels.contains(i))
         {
            adjacentLabels.add(i);
            LogTools.info("adjacent label " + i);
         }
      }

      return adjacentLabels.toArray();
   }

   public Point3D getLabelCenter(int label)
   {
      return segmentedData.get(label).getCenter();
   }

   public Vector3D getLabelNormal(int label)
   {
      return segmentedData.get(label).getNormal();
   }

   private class Segment
   {
      private final int label;
      private final TIntArrayList adjacentSegmentLabels = new TIntArrayList();
      private final List<Point3D> points = new ArrayList<>();

      private final Point3D center = new Point3D();
      private final Vector3D normal = new Vector3D();

      private final Vector3D standardDeviation = new Vector3D();

      private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

      private Segment(int labelID)
      {
         label = labelID;
      }

      private boolean contains(int otherLabel)
      {
         return adjacentSegmentLabels.contains(otherLabel);
      }

      private void addAdjacentSegmentLabel(int otherLabel)
      {
         adjacentSegmentLabels.add(otherLabel);
      }

      private void addPoint(Point3D point)
      {
         points.add(point);
      }

      private void update()
      {
         pca.clear();
         points.stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
         pca.compute();

         pca.getMean(center);
         pca.getThirdVector(normal);
         pca.getStandardDeviation(standardDeviation);
      }

      // TODO: handle if this label does not have enough number of points.
      private boolean isEmpty()
      {
         return points.size() < 4;
      }

      // TODO: do not merge when the label is sparse.
      // TODO: but,
      // TODO: at the last step of feature updater, each points of all the sparse labels should be separately merged to adjacent segment.
      private boolean isSparse(double threshold)
      {
         return standardDeviation.getZ() > threshold;
      }

      private int[] getAdjacentSegmentLabels()
      {
         return adjacentSegmentLabels.toArray();
      }

      private double getWeight()
      {
         return (double) points.size();
      }

      private Point3D getCenter()
      {
         return center;
      }

      private Vector3D getNormal()
      {
         return normal;
      }
   }
}
