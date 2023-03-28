package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class HeightMapObstacleDetector
{
   private static final double rSmall = 0.35;
   private static final double rLarge = 0.6;
   private static final double radialDropoff = 0.5;
   private static final double costSmall = 0.4;
   private static final double costLarge = 2.0;
   private static final double obstacleThreshold = 2.0;

   private final DMatrixRMaj mapRaw = new DMatrixRMaj(0);
   private final DMatrixRMaj mapFiltered = new DMatrixRMaj(0);
   private final DMatrixRMaj intensityGradientMagnitude = new DMatrixRMaj(0);
   private final DMatrixRMaj intensityGradientOrientation = new DMatrixRMaj(0);
   private final DMatrixRMaj terrainCost = new DMatrixRMaj(0);

   private final List<Vector2D> terrainOffsets = new ArrayList<>();
   private final TDoubleArrayList terrainOffsetCosts = new TDoubleArrayList();

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
      intensityGradientMagnitude.reshape(gridWidth, gridWidth);
      intensityGradientOrientation.reshape(gridWidth, gridWidth);
      terrainCost.reshape(gridWidth, gridWidth);

      CommonOps_DDRM.fill(mapRaw, 0.0);
      CommonOps_DDRM.fill(mapFiltered, 0.0);
      CommonOps_DDRM.fill(intensityGradientMagnitude, 0.0);
      CommonOps_DDRM.fill(intensityGradientOrientation, 0.0);
      CommonOps_DDRM.fill(terrainCost, 0.0);

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
            this.intensityGradientMagnitude.set(i, j, gInt);
            this.intensityGradientOrientation.set(i, j, gYaw);

            if (gInt > gIntMax)
               gIntMax = gInt;
         }
      }

      /* Computes traversibility */
      terrainOffsets.clear();
      terrainOffsetCosts.clear();
      int maxOffset = (int) Math.round(rLarge / heightMapData.getGridResolutionXY());

      for (int xi = -maxOffset; xi <= maxOffset; xi++)
      {
         for (int yi = -maxOffset; yi < maxOffset; yi++)
         {
            double x = xi * heightMapData.getGridResolutionXY();
            double y = yi * heightMapData.getGridResolutionXY();
            double theta = Math.atan2(y, x);
            double r = EuclidCoreTools.norm(x, y);
            double alpha = Math.abs(theta) / Math.PI;
            double rMax = EuclidCoreTools.interpolate(rSmall, rLarge, alpha);
            if (r <= rMax)
            {
               terrainOffsets.add(new Vector2D(x, y));
               terrainOffsetCosts.add(EuclidCoreTools.interpolate(1.0, radialDropoff, r / rMax) * EuclidCoreTools.interpolate(costSmall, costLarge, alpha));
            }
         }
      }

      for (int i = 2; i < gridWidth - 2; i++)
      {
         for (int j = 2; j < gridWidth - 2; j++)
         {
            if (intensityGradientMagnitude.get(i, j) < obstacleThreshold)
               continue;

            for (int k = 0; k < terrainOffsets.size(); k++)
            {
               Vector2D offset = new Vector2D(terrainOffsets.get(k));
               RigidBodyTransform transform = new RigidBodyTransform();
               transform.getRotation().setToYawOrientation(intensityGradientOrientation.get(i, j));
               offset.applyTransform(transform);

               int offsetX = (int) Math.round(offset.getX() / heightMapData.getGridResolutionXY());
               int offsetY = (int) Math.round(offset.getY() / heightMapData.getGridResolutionXY());
               int qX = i + offsetX;
               int qY = j + offsetY;
               if (qX < 0 || qY < 0 || qX >= gridWidth || qY >= gridWidth)
                  continue;

               terrainCost.set(qX, qY, Math.max(terrainCost.get(qX, qY), terrainOffsetCosts.get(k)));
            }
         }
      }
   }

   public DMatrixRMaj getEdgeDirectionMatrix()
   {
      return intensityGradientOrientation;
   }

   public DMatrixRMaj getEdgeIntensityMatrix()
   {
      return intensityGradientMagnitude;
   }

   public double getMaxEdgeIntensity()
   {
      return gIntMax;
   }

   public DMatrixRMaj getTerrainCost()
   {
      return terrainCost;
   }

   public static void main(String[] args) throws IOException
   {
      for (HeightMapDataSetName dataset : HeightMapDataSetName.values())
      {
         runObstacleDetector(dataset);
      }
   }

   private static void runObstacleDetector(HeightMapDataSetName dataset) throws IOException
   {
      HeightMapObstacleDetector obstacleDetector = new HeightMapObstacleDetector();
      obstacleDetector.compute(dataset.getHeightMapData());

      int width = 2 * dataset.getHeightMapData().getCenterIndex() + 1;
      BufferedImage unthresholded = new BufferedImage(width, width, BufferedImage.TYPE_INT_RGB);
      BufferedImage thresholded = new BufferedImage(width, width, BufferedImage.TYPE_INT_RGB);
      BufferedImage yaw = new BufferedImage(width, width, BufferedImage.TYPE_INT_RGB);
      BufferedImage cost = new BufferedImage(width, width, BufferedImage.TYPE_INT_RGB);
      double maxEdgeIntensity = obstacleDetector.getMaxEdgeIntensity();

      System.out.println(dataset.name() + ": " + maxEdgeIntensity);

      // speed up by checking neighbors of edge
      for (int i = 0; i < width; i++)
      {
         for (int j = 0; j < width; j++)
         {
            double intensityValue = obstacleDetector.getEdgeIntensityMatrix().get(i, j);
            int grayValue = (int) (255 * intensityValue / maxEdgeIntensity);
            unthresholded.setRGB(i, width - j - 1, new Color(grayValue, grayValue, grayValue, 255).getRGB());

            if (intensityValue > obstacleThreshold)
            {
               thresholded.setRGB(i, width - j - 1, new Color(grayValue, grayValue, grayValue, 255).getRGB());

               double yawAlpha = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(obstacleDetector.getEdgeDirectionMatrix().get(i, j), 0.0)) / Math.PI;
               grayValue = (int) (127 * yawAlpha) + 127;

               if (obstacleDetector.getEdgeDirectionMatrix().get(i, j) > 0.0)
                  yaw.setRGB(i, width - j - 1, new Color(grayValue, 0, 0, 255).getRGB());
               else
                  yaw.setRGB(i, width - j - 1, new Color(0, grayValue, 0, 255).getRGB());
            }
            else
            {
               thresholded.setRGB(i, width - j - 1, new Color(0, 0, 0, 255).getRGB());
               yaw.setRGB(i, width - j - 1, new Color(0, 0, 0, 255).getRGB());
            }
         }
      }

      for (int i = 0; i < width; i++)
      {
         for (int j = 0; j < width; j++)
         {
            int grayValue = (int) (255 * obstacleDetector.getTerrainCost().get(i, j) / costLarge);
            cost.setRGB(i, width - j - 1, new Color(grayValue, grayValue, grayValue, 255).getRGB());
         }
      }

      ImageIO.write(unthresholded, "png", new File(dataset.name() + ".png"));
      ImageIO.write(thresholded, "png", new File(dataset.name() + "_Thresh.png"));
      ImageIO.write(yaw, "png", new File(dataset.name() + "_Yaw.png"));
      ImageIO.write(cost, "png", new File(dataset.name() + "_Cost.png"));
   }
}
