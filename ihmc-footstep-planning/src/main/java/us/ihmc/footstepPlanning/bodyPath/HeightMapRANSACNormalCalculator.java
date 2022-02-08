package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class HeightMapRANSACNormalCalculator
{
   private static final int iterations = 120;
   private static final double ransacRadius = 0.16;
   private static final double consensusRadius = 0.12;
   private static final double distanceEpsilon = 0.032;
   private static final double maxAngleToConsider = Math.toRadians(65.0);
   private static final double minNormalZ = Math.acos(maxAngleToConsider);

   private UnitVector3DBasics[] surfaceNormals;
   private boolean[] hasComputedNormal;

   private HeightMapData heightMapData;
   private double gridResolution;
   private TIntArrayList xRansacOffsets;
   private TIntArrayList yRansacOffsets;
   private TIntArrayList xConsensusOffsets;
   private TIntArrayList yConsensusOffsets;

   private final Plane3D maxConsensusPlane = new Plane3D();
   private final Plane3D candidatePlane = new Plane3D();

   private final Point3D point = new Point3D();
   private final Point3D point0 = new Point3D();
   private final Point3D point1 = new Point3D();
   private final TIntArrayList samples = new TIntArrayList();

   private final Random random = new Random(3290);

   public void initialize(HeightMapData heightMapData)
   {
      if (xRansacOffsets == null || !EuclidCoreTools.epsilonEquals(gridResolution, heightMapData.getGridResolutionXY(), 1e-3))
      {
         gridResolution = heightMapData.getGridResolutionXY();
         int maxOffset = (int) Math.round(ransacRadius / heightMapData.getGridResolutionXY());

         xRansacOffsets = new TIntArrayList();
         yRansacOffsets = new TIntArrayList();
         xConsensusOffsets = new TIntArrayList();
         yConsensusOffsets = new TIntArrayList();

         for (int xi = -maxOffset; xi <= maxOffset; xi++)
         {
            for (int yi = -maxOffset; yi <= maxOffset; yi++)
            {
               if (heightMapData.getGridResolutionXY() * EuclidCoreTools.norm(xi, yi) < ransacRadius)
               {
                  xRansacOffsets.add(xi);
                  yRansacOffsets.add(yi);
               }
               if (heightMapData.getGridResolutionXY() * EuclidCoreTools.norm(xi, yi) < consensusRadius)
               {
                  xConsensusOffsets.add(xi);
                  yConsensusOffsets.add(yi);
               }
            }
         }

         LogTools.info("Offsets: " + xRansacOffsets.size());
      }

      this.heightMapData = heightMapData;
      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      hasComputedNormal = new boolean[gridWidth * gridWidth];
      surfaceNormals = new UnitVector3DBasics[gridWidth * gridWidth];
   }

   private void compute(int xIndex, int yIndex)
   {
      int centerIndex = heightMapData.getCenterIndex();

      if (heightMapData.isCellAtGroundPlane(xIndex, yIndex))
      {
         surfaceNormals[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)] = new UnitVector3D(Axis3D.Z);
         return;
      }

      int xOffset0, yOffset0, xOffset1, yOffset1;
      int xIndex0, yIndex0, xIndex1, yIndex1;

      int maxConsensus = -1;
      samples.clear();

      point.set(HeightMapTools.indexToCoordinate(xIndex, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex),
                HeightMapTools.indexToCoordinate(yIndex, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex),
                heightMapData.getHeightAt(xIndex, yIndex));

      for (int i = 0; i < iterations; i++)
      {
         while (true)
         {
            int sample0 = random.nextInt(xRansacOffsets.size());
            xOffset0 = xRansacOffsets.get(sample0);
            yOffset0 = yRansacOffsets.get(sample0);

            xIndex0 = xIndex + xOffset0;
            yIndex0 = yIndex + yOffset0;
            if (samples.contains(sample0) || xIndex0 < 0 || xIndex0 >= heightMapData.getCellsPerAxis() || yIndex0 < 0 || yIndex0 >= heightMapData.getCellsPerAxis())
            {
               continue;
            }

            int sample1 = random.nextInt(xRansacOffsets.size());
            if (sample0 == sample1)
            {
               continue;
            }

            xOffset1 = xRansacOffsets.get(sample1);
            yOffset1 = yRansacOffsets.get(sample1);

            xIndex1 = xIndex + xOffset1;
            yIndex1 = yIndex + yOffset1;
            if (xIndex1 < 0 || xIndex1 >= heightMapData.getCellsPerAxis() || yIndex1 < 0 || yIndex1 >= heightMapData.getCellsPerAxis())
            {
               continue;
            }

            double dotProduct = (xOffset0 * xOffset1 + yOffset0 * yOffset1) / (EuclidCoreTools.norm(xOffset0, yOffset0) * EuclidCoreTools.norm(xOffset1, yOffset1));
            if (EuclidCoreTools.epsilonEquals(Math.abs(dotProduct), 1.0, 1e-5))
            {
               continue;
            }

            break;
         }

         point0.set(HeightMapTools.indexToCoordinate(xIndex0, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex),
                    HeightMapTools.indexToCoordinate(yIndex0, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex),
                    heightMapData.getHeightAt(xIndex0, yIndex0));
         point1.set(HeightMapTools.indexToCoordinate(xIndex1, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex),
                    HeightMapTools.indexToCoordinate(yIndex1, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex),
                    heightMapData.getHeightAt(xIndex1, yIndex1));

         candidatePlane.set(point, point0, point1);

         if (Math.abs(candidatePlane.getNormal().getZ()) < minNormalZ)
         {
            continue;
         }

         int consensus = 0;
         for (int j = 0; j < xConsensusOffsets.size(); j++)
         {
            int xj = xIndex + xConsensusOffsets.get(j);
            int yj = yIndex + yConsensusOffsets.get(j);
            if (xj < 0 || xj >= heightMapData.getCellsPerAxis() || yj < 0 || yj >= heightMapData.getCellsPerAxis())
            {
               continue;
            }

            if (candidatePlane.distance(HeightMapTools.indexToCoordinate(xj, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(),
                                                                         centerIndex),
                                        HeightMapTools.indexToCoordinate(yj, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(),
                                                                         centerIndex),
                                        heightMapData.getHeightAt(xj, yj)) < distanceEpsilon)
            {
               consensus++;
            }
         }

         if (consensus > maxConsensus)
         {
            maxConsensus = consensus;
            maxConsensusPlane.set(candidatePlane);
         }
      }

      UnitVector3D surfaceNormal = new UnitVector3D(maxConsensusPlane.getNormal());
      if (surfaceNormal.getZ() < 0.0)
      {
         surfaceNormal.setX(-surfaceNormal.getX());
         surfaceNormal.setY(-surfaceNormal.getY());
         surfaceNormal.setZ(-surfaceNormal.getZ());
      }

      surfaceNormals[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)] = surfaceNormal;
   }

   public UnitVector3DBasics getSurfaceNormal(int key)
   {
      if (key < 0 || key >= surfaceNormals.length)
      {
         return null;
      }
      else if (!hasComputedNormal[key])
      {
         compute(HeightMapTools.keyToXIndex(key, heightMapData.getCenterIndex()), HeightMapTools.keyToYIndex(key, heightMapData.getCenterIndex()));
         hasComputedNormal[key] = true;
      }

      return surfaceNormals[key];
   }

   public UnitVector3DBasics getSurfaceNormal(int xIndex, int yIndex)
   {
      return getSurfaceNormal(HeightMapTools.indicesToKey(xIndex, yIndex, heightMapData.getCenterIndex()));
   }
}
