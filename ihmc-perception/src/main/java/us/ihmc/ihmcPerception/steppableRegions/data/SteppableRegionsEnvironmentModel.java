package us.ihmc.ihmcPerception.steppableRegions.data;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

public class SteppableRegionsEnvironmentModel
{
   int planarRegionIslandIndex = 0;

   private final HashSet<SteppableCell> unexpandedInteriorCellsInTheEnvironment = new HashSet<>();
   private final HashSet<SteppableCell> unexpandedBorderCellsInTheEnvironment = new HashSet<>();
   private final SteppableCell[][] steppableCellsGrid;
   private final int cellsPerSide;

   private final List<SteppableRegionDataHolder> steppableRegions = new ArrayList<>();

   public SteppableRegionsEnvironmentModel(int cellsPerSide)
   {
      this.cellsPerSide = cellsPerSide;
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
      return !unexpandedInteriorCellsInTheEnvironment.isEmpty();
   }

   public boolean hasUnexpandedBorderCells()
   {
      return !unexpandedBorderCellsInTheEnvironment.isEmpty();
   }

   public SteppableCell getNextUnexpandedInteriorCell()
   {
      SteppableCell cell = unexpandedInteriorCellsInTheEnvironment.stream().findFirst().get();
      cell.setCellHasBeenExpanded(true);
      return cell;
   }

   public SteppableCell getNextUnexpandedBorderCell()
   {
      SteppableCell cell = unexpandedBorderCellsInTheEnvironment.stream().findFirst().get();
      cell.setCellHasBeenExpanded(true);
      return cell;
   }

   public void markCellAsExpanded(SteppableCell cell)
   {
      if (cell.isBorderCell())
         unexpandedBorderCellsInTheEnvironment.remove(cell);
      else
         unexpandedInteriorCellsInTheEnvironment.remove(cell);
      cell.setCellHasBeenExpanded(true);
   }

   public void addUnexpandedSteppableCell(SteppableCell cell)
   {
      if (cell.isBorderCell())
         unexpandedBorderCellsInTheEnvironment.add(cell);
      else
         unexpandedInteriorCellsInTheEnvironment.add(cell);
      steppableCellsGrid[cell.getX()][cell.getY()] = cell;
   }

   public SteppableRegionDataHolder createNewSteppableRegion()
   {
      SteppableRegionDataHolder steppableRegion = new SteppableRegionDataHolder(planarRegionIslandIndex++);
      steppableRegions.add(steppableRegion);

      return steppableRegion;
   }
}
