package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import boofcv.struct.calib.IntrinsicParameters;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class LidarImageFusionRawData
{
   private final BufferedImage imageData;
   private final int imageWidth;
   private final int imageHeight;

   private final int[] labels;

   private final IntrinsicParameters intrinsicParameters;

   private final ArrayList<FusionDataSegment> fusionDataSegments = new ArrayList<FusionDataSegment>();

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

      LogTools.info("numberOfLabels " + numberOfLabels + ", numberOfPoints " + pointCloud.length);
      for (int i = 0; i < numberOfLabels; i++)
         fusionDataSegments.add(new FusionDataSegment(i));

      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, intrinsicParameters);
         int arrayIndex = getArrayIndex(pixel[0], pixel[1]);
         int label = labels[arrayIndex];

         fusionDataSegments.get(label).addPoint(new Point3D(point));
      }
   }

   private int getArrayIndex(int u, int v)
   {
      return u + v * imageWidth;
   }
   
   public void clear()
   {
      for(FusionDataSegment fusionDataSegment:fusionDataSegments)
         fusionDataSegment.setID(-1);
   }

   public void initializeSegments()
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
                  if (!fusionDataSegments.get(curLabel).contains(labelOfAdjacentPixel))
                     fusionDataSegments.get(curLabel).addAdjacentSegmentLabel(labelOfAdjacentPixel);
               }
            }
         }
      }

      for (FusionDataSegment fusionDataSegment : fusionDataSegments)
      {
         fusionDataSegment.update();
      }
   }

   public int getNumberOfLabels()
   {
      return fusionDataSegments.size();
   }

   public FusionDataSegment getFusionDataSegment(int label)
   {
      return fusionDataSegments.get(label);
   }

   public int[] getAdjacentLabels(TIntArrayList labels)
   {
      TIntArrayList uncompressedAdjacentLabels = new TIntArrayList();
      TIntArrayList adjacentLabels = new TIntArrayList();

      for (int label : labels.toArray())
      {
         uncompressedAdjacentLabels.addAll(fusionDataSegments.get(label).getAdjacentSegmentLabels());
      }

      for (int i = 0; i < fusionDataSegments.size(); i++)
      {
         if (uncompressedAdjacentLabels.contains(i) && !labels.contains(i))
         {
            adjacentLabels.add(i);
         }
      }

      return adjacentLabels.toArray();
   }

   public boolean allIdentified()
   {
      for (FusionDataSegment fusionDataSegment : fusionDataSegments)
      {
         if (fusionDataSegment.getId() == -1)
            return false;
      }
      return true;
   }
}
