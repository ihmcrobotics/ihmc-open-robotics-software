package us.ihmc.robotics.occupancyGrid;

import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class OccupancyGrid
{
   private final YoDouble decayRate;

   private final YoDouble cellXSize;
   private final YoDouble cellYSize;
   private final YoDouble cellArea;

   private final YoDouble thresholdForCellActivation;

   private final ReferenceFrame gridFrame;

   private final AtomicBoolean resetOccupancyGrid = new AtomicBoolean();

   final IntMap<OccupancyGridCell> occupancyCellMap = new IntMap<>();
   final List<OccupancyGridCell> allCellsPool = new ArrayList<>();
   private final List<OccupancyGridCell> allActiveCells = new ArrayList<>();

   public OccupancyGrid(String namePrefix, ReferenceFrame gridFrame, YoVariableRegistry parentRegistry)
   {
      this.gridFrame = gridFrame;

      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);

      cellXSize = new YoDouble(namePrefix + "CellXSize", registry);
      cellYSize = new YoDouble(namePrefix + "CellYSize", registry);
      cellArea = new YoDouble(namePrefix + "CellArea", registry);

      thresholdForCellActivation = new YoDouble(namePrefix + "ThresholdForCellOccupancy", registry);
      decayRate = new YoDouble(namePrefix + "CellOccupancyDecayRate", registry);

      setupChangedGridParameterListeners();

      parentRegistry.addChild(registry);
   }

   private void setupChangedGridParameterListeners()
   {
      VariableChangedListener changedGridSizeListener = (v) ->
      {
         cellArea.set(cellXSize.getDoubleValue() * cellYSize.getDoubleValue());
      };
      cellYSize.addVariableChangedListener(changedGridSizeListener);
      cellXSize.addVariableChangedListener(changedGridSizeListener);
      changedGridSizeListener.notifyOfVariableChange(null);
   }

   public void setThresholdForCellOccupancy(double thresholdForCellOccupancy)
   {
      this.thresholdForCellActivation.set(thresholdForCellOccupancy);
   }

   public void setOccupancyDecayRate(double occupancyDecayRate)
   {
      this.decayRate.set(occupancyDecayRate);
   }

   public void setCellXSize(double xSize)
   {
      this.cellXSize.set(xSize);
      resetOccupancyGrid.set(true);
   }

   public void setCellYSize(double ySize)
   {
      this.cellYSize.set(ySize);
      resetOccupancyGrid.set(true);
   }

   public double getCellXSize()
   {
      return cellXSize.getDoubleValue();
   }

   public double getCellYSize()
   {
      return cellYSize.getDoubleValue();
   }

   public List<OccupancyGridCell> getAllActiveCells()
   {
      return allActiveCells;
   }

   public ReferenceFrame getGridFrame()
   {
      return gridFrame;
   }

   public void reset()
   {
      allActiveCells.clear();
      for (int i = 0; i < allCellsPool.size(); i++)
         allCellsPool.get(i).reset();
   }

   public void update()
   {
      if (resetOccupancyGrid.getAndSet(false))
         reset();

      for (int i = 0; i < allCellsPool.size(); i++)
         allCellsPool.get(i).update();
   }

   public void registerPoint(FramePoint2DReadOnly point)
   {
      point.checkReferenceFrameMatch(gridFrame);
      getOrCreateOccupancyGridCell(point).registerHit();
   }

   public boolean isCellOccupied(int xIndex, int yIndex)
   {
      return getOrCreateOccupancyGridCell(xIndex, yIndex).getIsOccupied();
   }

   public double getXLocation(int xIndex)
   {
      return cellXSize.getDoubleValue() * xIndex;
   }

   public double getYLocation(int yIndex)
   {
      return cellYSize.getDoubleValue() * yIndex;
   }

   private int findXIndex(double x)
   {
      return (int) Math.floor(x / cellXSize.getDoubleValue());
   }

   private int findYIndex(double y)
   {
      return (int) Math.floor(y / cellYSize.getDoubleValue());
   }

   private OccupancyGridCell getOrCreateOccupancyGridCell(Point2DReadOnly pointInGrid)
   {
      int xIndex = findXIndex(pointInGrid.getX());
      int yIndex = findYIndex(pointInGrid.getY());

      return getOrCreateOccupancyGridCell(xIndex, yIndex);
   }

   private OccupancyGridCell getOrCreateOccupancyGridCell(int xIndex, int yIndex)
   {
      int hashCode = OccupancyGridCell.computeHashCode(xIndex, yIndex);
      OccupancyGridCell cell = occupancyCellMap.get(hashCode);
      if (cell == null)
      {
         cell = new OccupancyGridCell(xIndex, yIndex, thresholdForCellActivation, decayRate);
         allCellsPool.add(cell);
         occupancyCellMap.put(hashCode, cell);
      }
      if (!allActiveCells.contains(cell))
         allActiveCells.add(cell);

      return cell;
   }
}
