package us.ihmc.perception.graphicalSegmentation;

import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.rapidRegions.*;

import java.util.Comparator;
import java.util.Stack;

public class GraphicalSegmentationCalculator
{
   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;

   private RapidRegionsExtractorParameters parameters;

   private final int[] adjacentY = {-1, -1, -1, 0, 0, 1, 1, 1};
   private final int[] adjacentX = {-1, 0, 1, -1, 1, -1, 0, 1};

   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesTotal = 0;
   private double maxSVDSolveTime = Double.NaN;

   private int patchImageHeight;
   private int patchImageWidth;
   private int patchHeight;
   private int patchWidth;

   private final Comparator<RapidRegionRing> boundaryLengthComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());
   private final Stack<PatchGraphRecursionBlock> depthFirstSearchStack = new Stack<>();
   private PatchFeatureGrid currentFeatureGrid;
   private RapidPatchesDebugOutputGenerator debugger;
   private BytedecoImage patchGraph;

   public GraphicalSegmentationCalculator(RapidRegionsExtractorParameters parameters,
                                          PatchFeatureGrid featureGrid,
                                          BytedecoImage patchGraph,
                                          RapidPatchesDebugOutputGenerator debugger)
   {
      this.patchGraph = patchGraph;
      this.debugger = debugger;
      this.currentFeatureGrid = featureGrid;
      this.parameters = parameters;
      this.patchImageHeight = featureGrid.getRows();
      this.patchImageWidth = featureGrid.getColumns();
      this.patchHeight = parameters.getPatchSize();
      this.patchWidth = parameters.getPatchSize();
      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
   }

   /**
    * Finds the connected regions in the patch graph using Depth First Search. It uses a heap-allocated stack object instead of process recursion stack.
    */
   public void findRegions(RecyclingArrayList<RapidPlanarRegion> rapidPlanarRegions)
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      rapidPlanarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      maxSVDSolveTime = 0.0;

      for (int row = 0; row < patchImageHeight; row++)
      {
         for (int column = 0; column < patchImageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = patchGraph.getByteAsInteger(row, column);

            if (!regionVisitedMatrix.get(row, column) && checkConnectionThreshold(boundaryConnectionsEncodedAsOnes,
                                                                                  parameters.getConnectionThreshold())) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               RapidPlanarRegion planarRegion = rapidPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);

               // Push the first call on stack
               depthFirstSearchStack.push(new PatchGraphRecursionBlock(row, column, planarRegionIslandIndex, planarRegion, 1));

               // Loop until a new connected region has been found
               while (!depthFirstSearchStack.empty())
               {
                  depthFirstSearchStack.pop().expandBlock();
               }

               // Create final rapid region if the connected island has enough patches
               //LogTools.info("Min Patch Count: {} | Number of Patches: {} | Island: {}", 20, numberOfRegionPatches, planarRegionIslandIndex);
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  //LogTools.info("Region Found: {}", planarRegionIslandIndex);
                  planarRegionIslandIndex++;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();
               }
               else
               {
                  int totalGPURegions = rapidPlanarRegions.size();
                  if (totalGPURegions > 0)
                  {
                     rapidPlanarRegions.remove(rapidPlanarRegions.size() - 1);
                  }
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
   }

   public void findBoundariesAndHoles(RecyclingArrayList<RapidPlanarRegion> rapidPlanarRegions)
   {
      boundaryVisitedMatrix.zero();
      boundaryMaxSearchDepth = 0;
      rapidPlanarRegions.parallelStream().forEach(planarRegion ->
        {
           int leafPatchIndex = 0;
           int regionRingIndex = 0;
           planarRegion.getRegionsRingsBySize().clear();
           for (Point2D leafPatch : planarRegion.getBorderIndices())
           {
              RapidRegionRing regionRing = planarRegion.getRegionRings().add();
              regionRing.reset();
              regionRing.setIndex(regionRingIndex);
              int numberOfBoundaryPatches = boundaryDepthFirstSearch((int) leafPatch.getY(),
                                                                     (int) leafPatch.getX(),
                                                                     planarRegion.getId(),
                                                                     regionRing,
                                                                     leafPatchIndex,
                                                                     1);
              if (numberOfBoundaryPatches >= parameters.getBoundaryMinPatches())
              {
                 //debugger.drawRegionRing(regionRing, patchHeight, patchWidth);

                 ++regionRingIndex;
                 regionRing.updateConvexPolygon();
                 planarRegion.getRegionsRingsBySize().add(regionRing);
              }
              else
              {
                 planarRegion.getRegionRings().remove(planarRegion.getRegionRings().size() - 1);
              }
              ++leafPatchIndex;
           }

           // remove holes
           for (RapidRegionRing regionRing : planarRegion.getRegionsRingsBySize())
           {
              planarRegion.getHoleRingsToRemove().clear();
              for (RapidRegionRing otherRegionRing : planarRegion.getRegionRings())
              {
                 if (otherRegionRing != regionRing)
                 {
                    // We probably only need to check one
                    Vector2D boundaryIndex = otherRegionRing.getBoundaryIndices().get(0);
                    if (regionRing.getConvexPolygon().isPointInside(boundaryIndex.getX(), boundaryIndex.getY()))
                    {
                       planarRegion.getHoleRingsToRemove().add(otherRegionRing);
                    }
                 }
              }
              for (RapidRegionRing regionRingToRemove : planarRegion.getHoleRingsToRemove())
              {
                 planarRegion.getRegionRings().remove(regionRingToRemove);
              }
           }

           planarRegion.getRegionRings().sort(boundaryLengthComparator);
        });
   }

   private int boundaryDepthFirstSearch(int row, int column, int planarRegionId, RapidRegionRing regionRing, int leafPatchIndex, int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > parameters.getBoundarySearchDepthLimit())
         return 0;

      if (searchDepth > boundaryMaxSearchDepth)
         boundaryMaxSearchDepth = searchDepth;

      ++numberOfBoundaryPatchesTotal;
      boundaryVisitedMatrix.set(row, column, true);
      regionRing.getBoundaryIndices().add().set(column, row);

      int numberOfBoundaryPatches = 1;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1
             && boundaryMatrix.get(row + adjacentY[i], column + adjacentX[i]) && planarRegionId == regionMatrix.get(row + adjacentY[i], column + adjacentX[i]))
         {
            numberOfBoundaryPatches += boundaryDepthFirstSearch(row + adjacentY[i],
                                                                column + adjacentX[i],
                                                                planarRegionId,
                                                                regionRing,
                                                                leafPatchIndex,
                                                                searchDepth + 1);
         }
      }
      return numberOfBoundaryPatches;
   }

   public void reshape(int patchImageHeight, int patchImageWidth)
   {
      regionVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
      boundaryMatrix.reshape(patchImageHeight, patchImageWidth);
      regionMatrix.reshape(patchImageHeight, patchImageWidth);
   }

   public boolean checkConnectionNonZero(int nodeConnection)
   {
      return Integer.bitCount(nodeConnection) > 0;
   }

   public boolean checkConnectionFull(int nodeConnection)
   {
      return nodeConnection == 255;
   }

   public boolean checkConnectionThreshold(int nodeConnection, int threshold)
   {
      return Integer.bitCount(nodeConnection) > threshold;
   }

   public boolean checkConnectionDirectional(int nodeConnection, int neighbor)
   {
      int mask = 1 << neighbor;
      return (nodeConnection & mask) != 0;
   }

   public int getNumberOfBoundaryPatchesTotal()
   {
      return numberOfBoundaryPatchesTotal;
   }

   public int getNumberOfRegionPatches()
   {
      return numberOfRegionPatches;
   }

   public int getRegionMaxSearchDepth()
   {
      return regionMaxSearchDepth;
   }

   public int getBoundaryMaxSearchDepth()
   {
      return boundaryMaxSearchDepth;
   }

   public double getMaxSVDSolveTime()
   {
      return maxSVDSolveTime;
   }

   public class PatchGraphRecursionBlock
   {
      private final int row;
      private final int column;
      private final int planarRegionIslandIndex;
      private final RapidPlanarRegion planarRegion;
      private final int searchDepth;

      public PatchGraphRecursionBlock(int row, int column, int planarRegionIslandIndex, RapidPlanarRegion planarRegion, int searchDepth)
      {
         this.row = row;
         this.column = column;
         this.planarRegionIslandIndex = planarRegionIslandIndex;
         this.planarRegion = planarRegion;
         this.searchDepth = searchDepth;
      }

      public void expandBlock()
      {
         if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getInternalSearchDepthLimit())
            return;

         LogTools.debug("Expanding block at row: {}, column: {}, searchDepth: {}", row, column, searchDepth);

         if (searchDepth > regionMaxSearchDepth)
            regionMaxSearchDepth = searchDepth;

         ++numberOfRegionPatches;
         regionVisitedMatrix.set(row, column, true);
         regionMatrix.set(row, column, planarRegionIslandIndex);

         float nx = currentFeatureGrid.getNxImage().getFloat(row, column);
         float ny = currentFeatureGrid.getNyImage().getFloat(row, column);
         float nz = currentFeatureGrid.getNzImage().getFloat(row, column);
         float cx = currentFeatureGrid.getCxImage().getFloat(row, column);
         float cy = currentFeatureGrid.getCyImage().getFloat(row, column);
         float cz = currentFeatureGrid.getCzImage().getFloat(row, column);

         planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

         debugger.drawInternalNode(planarRegionIslandIndex, column, row, patchHeight, patchWidth);

         int count = 0;
         for (int i = 0; i < 8; i++)
         {
            if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
            {
               int boundaryConnectionsEncodedAsOnes = patchGraph.getByteAsInteger((row + adjacentY[i]), (column + adjacentX[i]));
               if (checkConnectionThreshold(boundaryConnectionsEncodedAsOnes, parameters.getConnectionThreshold())) // all ones; fully connected
               {
                  ++count;
                  depthFirstSearchStack.push(new PatchGraphRecursionBlock(row + adjacentY[i],
                                                                          column + adjacentX[i],
                                                                          planarRegionIslandIndex,
                                                                          planarRegion,
                                                                          searchDepth + 1));
               }
            }
         }
         if (count != 8)
         {
            boundaryMatrix.set(row, column, true);
            Point2D boundaryPoint = planarRegion.getBorderIndices().add();

            if (boundaryPoint != null)
               boundaryPoint.set(column, row);
            //debugger.drawBoundaryNode(planarRegionIslandIndex, column, row, patchHeight, patchWidth);
         }
      }
   }
}
