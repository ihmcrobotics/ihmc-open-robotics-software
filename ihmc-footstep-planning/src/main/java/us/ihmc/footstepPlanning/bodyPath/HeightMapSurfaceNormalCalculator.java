package us.ihmc.footstepPlanning.bodyPath;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class HeightMapSurfaceNormalCalculator
{
   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private UnitVector3DBasics[] surfaceNormals;

   private final DMatrixRMaj mapRaw = new DMatrixRMaj(0);
   private final DMatrixRMaj mapBlurFiltered = new DMatrixRMaj(0);

   private static final int[] xBlurOffsets = new int[]{-1, 0, 1, -1, 0, 1, -1, 0, 1};
   private static final int[] yBlurOffsets = new int[]{1, 1, 1, 0, 0, 0, -1, -1, -1};
   private static final double[] blurKernel = new double[]{1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0};

   static
   {
      for (int i = 0; i < blurKernel.length; i++)
      {
         blurKernel[i] = blurKernel[i] / 16.0;
      }
   }

   public void computeSurfaceNormals(HeightMapData heightMapData, double patchWidth)
   {
      applyBlurFilter(heightMapData);

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
      polygonToSnap.addVertex(-0.5 * patchWidth, 0.5 * patchWidth);
      polygonToSnap.addVertex(-0.5 * patchWidth, -0.5 * patchWidth);
      polygonToSnap.addVertex(0.5 * patchWidth, 0.5 * patchWidth);
      polygonToSnap.addVertex(0.5 * patchWidth, -0.5 * patchWidth);
      polygonToSnap.update();

      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      surfaceNormals = new UnitVector3DBasics[gridWidth * gridWidth];
      int centerIndex = heightMapData.getCenterIndex();

      for (int xIndex = 0; xIndex < gridWidth; xIndex++)
      {
         for (int yIndex = 0; yIndex < gridWidth; yIndex++)
         {
            UnitVector3DBasics surfaceNormal = new UnitVector3D(0.0, 0.0, 1.0);
            surfaceNormals[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)] = surfaceNormal;

            if (xIndex > 0 && yIndex > 0 && xIndex < gridWidth - 1 && yIndex < gridWidth - 1)
            {
               ConvexPolygon2D translatedPatch = new ConvexPolygon2D(polygonToSnap);
               translatedPatch.translate(HeightMapTools.indexToCoordinate(xIndex,
                                                                          heightMapData.getGridCenter().getX(),
                                                                          heightMapData.getGridResolutionXY(),
                                                                          heightMapData.getCenterIndex()),
                                         HeightMapTools.indexToCoordinate(yIndex,
                                                                          heightMapData.getGridCenter().getY(),
                                                                          heightMapData.getGridResolutionXY(),
                                                                          heightMapData.getCenterIndex()));

               heightMapSnapper.snapPolygonToHeightMap(translatedPatch, heightMapData);
               surfaceNormal.set(heightMapSnapper.getBestFitPlane().getNormal());
            }
         }
      }
   }

   private void applyBlurFilter(HeightMapData heightMapData)
   {
      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      mapRaw.reshape(gridWidth, gridWidth);
      mapBlurFiltered.reshape(gridWidth, gridWidth);

      CommonOps_DDRM.fill(mapRaw, 0.0);
      CommonOps_DDRM.fill(mapBlurFiltered, 0.0);

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
            for (int k = 0; k < xBlurOffsets.length; k++)
            {
               filteredHeight += blurKernel[k] * heightMapData.getHeightAt(i + xBlurOffsets[k], j + yBlurOffsets[k]);
            }

            mapBlurFiltered.set(i, j, filteredHeight);
         }
      }
   }

   public UnitVector3DBasics getSurfaceNormal(int key)
   {
      if (key >= 0 && key < surfaceNormals.length)
      {
         return surfaceNormals[key];
      }
      else
      {
         return null;
      }
   }
}
