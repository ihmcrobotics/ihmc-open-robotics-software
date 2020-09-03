package us.ihmc.robotics.occupancyGrid;

import com.esotericsoftware.kryo.util.IntMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class OccupancyGrid
{
   static final double defaultCellSize = 0.01;
   private static final double defaultNoDecay = 0.0;
   private static final double defaultOneHitOccupancy = 1.0;

   private final YoDouble decayRate;

   private final YoDouble cellXSize;
   private final YoDouble cellYSize;
   private final YoDouble cellArea;
   private final YoInteger numberOfOccupiedCells;

   private final YoDouble thresholdForCellActivation;

   private final ReferenceFrame gridFrame;

   private final AtomicBoolean resetOccupancyGrid = new AtomicBoolean();

   final IntMap<OccupancyGridCell> occupancyCellMap = new IntMap<>();
   final List<OccupancyGridCell> allCellsPool = new ArrayList<>();
   private final List<OccupancyGridCell> allActiveCells = new ArrayList<>();

   public OccupancyGrid(String namePrefix, ReferenceFrame gridFrame, YoRegistry parentRegistry)
   {
      this.gridFrame = gridFrame;

      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(namePrefix + name);

      cellXSize = new YoDouble(namePrefix + "CellXSize", registry);
      cellYSize = new YoDouble(namePrefix + "CellYSize", registry);
      cellArea = new YoDouble(namePrefix + "CellArea", registry);
      numberOfOccupiedCells = new YoInteger(namePrefix + "NumberOfOccupiedCells", registry);

      thresholdForCellActivation = new YoDouble(namePrefix + "ThresholdForCellOccupancy", registry);
      decayRate = new YoDouble(namePrefix + "CellOccupancyDecayRate", registry);

      thresholdForCellActivation.set(defaultOneHitOccupancy);
      decayRate.set(defaultNoDecay);
      cellXSize.set(defaultCellSize);
      cellYSize.set(defaultCellSize);

      setupChangedGridParameterListeners();

      parentRegistry.addChild(registry);
   }

   private void setupChangedGridParameterListeners()
   {
      YoVariableChangedListener changedGridSizeListener = (v) ->
      {
         cellArea.set(cellXSize.getDoubleValue() * cellYSize.getDoubleValue());
      };
      cellYSize.addListener(changedGridSizeListener);
      cellXSize.addListener(changedGridSizeListener);
      changedGridSizeListener.changed(null);
   }

   public void setThresholdForCellOccupancy(double thresholdForCellOccupancy)
   {
      this.thresholdForCellActivation.set(thresholdForCellOccupancy);
   }

   public void setOccupancyDecayRate(double occupancyDecayRate)
   {
      this.decayRate.set(occupancyDecayRate);
   }

   public void setCellSize(double size)
   {
      setCellXSize(size);
      setCellYSize(size);
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
      numberOfOccupiedCells.set(0);
      allActiveCells.clear();
      for (int i = 0; i < allCellsPool.size(); i++)
         allCellsPool.get(i).reset();
   }

   public void update()
   {
      if (resetOccupancyGrid.getAndSet(false))
         reset();

      numberOfOccupiedCells.set(0);
      for (int i = 0; i < allCellsPool.size(); i++)
      {
         if (allCellsPool.get(i).update())
            numberOfOccupiedCells.increment();
      }
   }

   public int registerPoint(FramePoint2DReadOnly point)
   {
      point.checkReferenceFrameMatch(gridFrame);
      OccupancyGridCell cell = getOrCreateOccupancyGridCell(point);
      boolean wasOccupied = cell.getIsOccupied();

      if (cell.registerHit() && !wasOccupied)
         numberOfOccupiedCells.increment();

      return numberOfOccupiedCells.getIntegerValue();
   }

   public int getNumberOfOccupiedCells()
   {
      return numberOfOccupiedCells.getIntegerValue();
   }

   public boolean isCellOccupied(double x, double y)
   {

      return isCellOccupied(findXIndex(x), findYIndex(y));
   }

   public boolean isCellOccupied(int xIndex, int yIndex)
   {
      OccupancyGridCell cell = getCell(xIndex, yIndex);
      if (cell != null)
         return cell.getIsOccupied();
      return false;
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
      return findIndex(x, cellXSize.getDoubleValue());
   }

   private int findYIndex(double y)
   {
      return findIndex(y, cellYSize.getDoubleValue());
   }

   static int findIndex(double value, double gridSize)
   {
      return (int) Math.floor(value / gridSize);
   }

   private OccupancyGridCell getOrCreateOccupancyGridCell(Point2DReadOnly pointInGrid)
   {
      int xIndex = findXIndex(pointInGrid.getX());
      int yIndex = findYIndex(pointInGrid.getY());

      return getOrCreateOccupancyGridCell(xIndex, yIndex);
   }

   private OccupancyGridCell getCell(int xIndex, int yIndex)
   {
      int hashCode = OccupancyGridCell.computeHashCode(xIndex, yIndex);
      return occupancyCellMap.get(hashCode);
   }

   private OccupancyGridCell createCell(int xIndex, int yIndex)
   {
      int hashCode = OccupancyGridCell.computeHashCode(xIndex, yIndex);
      OccupancyGridCell cell = new OccupancyGridCell(xIndex, yIndex, thresholdForCellActivation, decayRate);
      allCellsPool.add(cell);
      occupancyCellMap.put(hashCode, cell);

      return cell;
   }

   private OccupancyGridCell getOrCreateOccupancyGridCell(int xIndex, int yIndex)
   {
      OccupancyGridCell cell = getCell(xIndex, yIndex);
      if (cell == null)
         cell = createCell(xIndex, yIndex);

      if (!allActiveCells.contains(cell))
         allActiveCells.add(cell);

      return cell;
   }
}
