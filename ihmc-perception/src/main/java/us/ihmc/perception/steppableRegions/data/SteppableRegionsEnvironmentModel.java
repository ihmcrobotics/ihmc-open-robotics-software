package us.ihmc.perception.steppableRegions.data;

import org.ejml.data.DMatrixRMaj;

import java.util.*;

public class SteppableRegionsEnvironmentModel
{
   int planarRegionIslandIndex = 0;

   private final LinkedList<SteppableCell> unexpandedBorderCellsInTheEnvironment = new LinkedList<>();
   private final SteppableCell[][] steppableCellsGrid;
   private final int cellsPerSide;

   private final List<SteppableRegionDataHolder> steppableRegions = new ArrayList<>();

   private int activeRowIndex = 0;
   private int activeColumnIndex = 0;
   private final DMatrixRMaj cellVisitedMatrix;

   public SteppableRegionsEnvironmentModel(int cellsPerSide)
   {
      this.cellsPerSide = cellsPerSide;
      cellVisitedMatrix = new DMatrixRMaj(cellsPerSide, cellsPerSide);

      steppableCellsGrid = new SteppableCell[cellsPerSide][];
      for (int i = 0; i < cellsPerSide; i++)
         steppableCellsGrid[i] = new SteppableCell[cellsPerSide];
   }

   public int getCellsPerSide()
   {
      return cellsPerSide;
   }

   public SteppableCell getCellAt(int x, int y)
   {
      return steppableCellsGrid[x][y];
   }

   public Collection<SteppableRegionDataHolder> getRegions()
   {
      return steppableRegions;
   }

   public void removeRegion(SteppableRegionDataHolder region)
   {
      region.clear();
      steppableRegions.remove(region);
   }

   public boolean hasUnexpandedInteriorCells()
   {
      return activeRowIndex != cellsPerSide;
   }

   public boolean hasUnexpandedBorderCells()
   {
      return !unexpandedBorderCellsInTheEnvironment.isEmpty();
   }

   public SteppableCell getNextUnexpandedCell()
   {
      while (activeRowIndex < cellsPerSide)
      {
         while (activeColumnIndex < cellsPerSide)
         {
            if (steppableCellsGrid[activeRowIndex][activeColumnIndex] == null)
            {
               cellVisitedMatrix.set(activeRowIndex, activeColumnIndex, 1.0);
            }
            else if (cellVisitedMatrix.get(activeRowIndex, activeColumnIndex) == 0.0)
            {
               cellVisitedMatrix.set(activeRowIndex, activeColumnIndex, 1.0);
               SteppableCell cell = steppableCellsGrid[activeRowIndex][activeColumnIndex];
               cell.setCellHasBeenExpanded(true);
               activeColumnIndex++;
               return cell;
            }
            // it's either unused (i.e. null) or we've already searched here, so advance to the next column.
            activeColumnIndex++;
         }
         // advance to the next row.
         activeRowIndex++;
         activeColumnIndex = 0;
      }

      // this is done to demarcate that we're done with the search.
      activeColumnIndex = cellsPerSide;
      return null;
   }

   public SteppableCell getNextUnexpandedBorderCell()
   {
      SteppableCell cell = unexpandedBorderCellsInTheEnvironment.get(0);
      cellVisitedMatrix.set(cell.getXIndex(), cell.getYIndex(), 1.0);
      cell.setCellHasBeenExpanded(true);
      return cell;
   }

   public void markCellAsExpanded(SteppableCell cell)
   {
      if (cell.isBorderCell())
         unexpandedBorderCellsInTheEnvironment.remove(cell);

      cellVisitedMatrix.set(cell.getXIndex(), cell.getYIndex(), 1.0);
      cell.setCellHasBeenExpanded(true);
   }

   public void addUnexpandedSteppableCell(SteppableCell cell)
   {
      if (cell.isBorderCell())
         unexpandedBorderCellsInTheEnvironment.add(cell);

      steppableCellsGrid[cell.getXIndex()][cell.getYIndex()] = cell;
   }

   public SteppableRegionDataHolder createNewSteppableRegion()
   {
      SteppableRegionDataHolder steppableRegion = new SteppableRegionDataHolder(planarRegionIslandIndex++);
      steppableRegions.add(steppableRegion);

      return steppableRegion;
   }
}
