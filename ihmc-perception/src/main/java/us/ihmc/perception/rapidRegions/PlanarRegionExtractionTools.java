package us.ihmc.perception.rapidRegions;

import java.util.function.Consumer;

public class PlanarRegionExtractionTools
{
   public void findRegions(Consumer<GPUPlanarRegionIsland> forDrawingDebugPanel)
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      gpuPlanarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      maxSVDSolveTime = 0.0;
      for (int row = 0; row < patchImageHeight; row++)
      {
         for (int column = 0; column < patchImageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr(row, column).get());
            if (!regionVisitedMatrix.get(row, column) && boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               GPUPlanarRegion planarRegion = gpuPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);
               regionsDepthFirstSearch(row, column, planarRegionIslandIndex, planarRegion, 1);
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  tempIsland.planarRegion = planarRegion;
                  tempIsland.planarRegionIslandIndex = planarRegionIslandIndex;
                  forDrawingDebugPanel.accept(tempIsland);
               }
               else
               {
                  gpuPlanarRegions.remove(gpuPlanarRegions.size() - 1);
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
   }

   private void regionsDepthFirstSearch(int row, int column, int planarRegionIslandIndex, GPUPlanarRegion planarRegion, int searchDepth)
   {
      if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
         return;

      if (searchDepth > regionMaxSearchDepth)
         regionMaxSearchDepth = searchDepth;

      ++numberOfRegionPatches;
      regionVisitedMatrix.set(row, column, true);
      regionMatrix.set(row, column, planarRegionIslandIndex);
      // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
      float ny = -nxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float nz = nyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float nx = nzImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cy = -cxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cz = cyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cx = czImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

      int count = 0;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
         {
            int boundaryConnectionsEncodedAsOnes
                  = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr((row + adjacentY[i]), (column + adjacentX[i])).get());
            if (boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               ++count;
               regionsDepthFirstSearch(row + adjacentY[i], column + adjacentX[i], planarRegionIslandIndex, planarRegion, searchDepth + 1);
            }
         }
      }
      if (count != 8)
      {
         boundaryMatrix.set(row, column, true);
         planarRegion.getBorderIndices().add().set(column, row);
      }
   }

}
