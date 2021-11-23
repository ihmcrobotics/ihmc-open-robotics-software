package us.ihmc.footstepPlanning.bodyPath;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class HeightMapObstacleDetector
{
   private final DMatrixRMaj mapRaw = new DMatrixRMaj(0);
   private final DMatrixRMaj mapFiltered = new DMatrixRMaj(0);
   private final DMatrixRMaj gInt = new DMatrixRMaj(0);
   private final DMatrixRMaj gYaw = new DMatrixRMaj(0);

   private static final int[] xOffsets = new int[]{-1, 0, 1, -1, 0, 1, -1, 0, 1};
   private static final int[] yOffsets = new int[]{1, 1, 1, 0, 0, 0, -1, -1, -1};

   private static final double[] blurKernel = new double[]{1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0};
   private static final double[] gxKernel = new double[]{-1.0, 0.0, 1.0, -2.0, 0.0, 2.0, -1.0, 0.0, 1.0};
   private static final double[] gyKernel = new double[]{1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0};

   private double gIntMax;

   static
   {
      for (int i = 0; i < blurKernel.length; i++)
         blurKernel[i] = blurKernel[i] / 16.0;
   }

   public void compute(HeightMapData heightMapData)
   {
      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      mapRaw.reshape(gridWidth, gridWidth);
      mapFiltered.reshape(gridWidth, gridWidth);
      gInt.reshape(gridWidth, gridWidth);
      gYaw.reshape(gridWidth, gridWidth);

      CommonOps_DDRM.fill(mapRaw, 0.0);
      CommonOps_DDRM.fill(mapFiltered, 0.0);
      CommonOps_DDRM.fill(gInt, 0.0);
      CommonOps_DDRM.fill(gYaw, 0.0);

      /* Populate raw map */
      double estimatedGroundHeight = heightMapData.getEstimatedGroundHeight();
      for (int xIndex = 0; xIndex < gridWidth; xIndex++)
      {
         for (int yIndex = 0; yIndex < gridWidth; yIndex++)
         {
            double height = heightMapData.getHeightAt(xIndex, yIndex);
            mapRaw.set(xIndex, yIndex, height - estimatedGroundHeight);
         }
      }

      /* Compute blur filter to decrease noise */
      for (int i = 1; i < gridWidth - 1; i++)
      {
         for (int j = 1; j < gridWidth - 1; j++)
         {
            double filteredHeight = 0.0;
            for (int k = 0; k < xOffsets.length; k++)
            {
               filteredHeight += blurKernel[k] * heightMapData.getHeightAt(i + xOffsets[k], j + yOffsets[k]);
            }

            mapFiltered.set(i, j, filteredHeight);
         }
      }

      /* Compute intensity gradients */
      gIntMax = -1.0;
      for (int i = 2; i < gridWidth - 2; i++)
      {
         for (int j = 2; j < gridWidth - 2; j++)
         {
            double gx_i = 0.0;
            double gy_i = 0.0;
            for (int k = 0; k < xOffsets.length; k++)
            {
               gx_i += gxKernel[k] * heightMapData.getHeightAt(i + xOffsets[k], j + yOffsets[k]);
               gy_i += gyKernel[k] * heightMapData.getHeightAt(i + xOffsets[k], j + yOffsets[k]);
            }

            double gInt = EuclidCoreTools.norm(gx_i, gy_i);
            double gYaw = Math.atan2(gy_i, gx_i);
            this.gInt.set(i, j, gInt);
            this.gYaw.set(i, j, gYaw);

            if (gInt > gIntMax)
               gIntMax = gInt;
         }
      }
   }

   public DMatrixRMaj getEdgeIntensityMatrix()
   {
      return gInt;
   }

   public double getMaxEdgeIntensity()
   {
      return gIntMax;
   }

   public static void main(String[] args) throws IOException
   {
      HeightMapDataSetName stairs = HeightMapDataSetName.Stepping_Stones_2;

      HeightMapObstacleDetector obstacleDetector = new HeightMapObstacleDetector();
      obstacleDetector.compute(stairs.getHeightMapData());

      int width = 2 * stairs.getHeightMapData().getCenterIndex() + 1;
      BufferedImage bufferedImage = new BufferedImage(width, width, BufferedImage.TYPE_INT_RGB);
      double maxEdgeIntensity = obstacleDetector.getMaxEdgeIntensity();

      for (int i = 0; i < width; i++)
      {
         for (int j = 0; j < width; j++)
         {
            int grayValue = (int) (255 * obstacleDetector.getEdgeIntensityMatrix().get(i, j) / maxEdgeIntensity);
            bufferedImage.setRGB(i, j, new Color(grayValue, grayValue, grayValue, 255).getRGB());
         }
      }

      File outputfile = new File(stairs.name() + ".png");
      ImageIO.write(bufferedImage, "png", outputfile);
   }
}
