package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridCell;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class FootCoPHullCropper
{
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 1.0;
   private static final double defaultAreaRatioThreshold = 0.6;

   private final YoInteger nLengthSubdivisions;
   private final YoInteger nWidthSubdivisions;

   private final ReferenceFrame soleFrame;

   private final double footLength;
   private final double footWidth;

   private final YoDouble areaOnRightSideOfLine;
   private final YoDouble areaOnLeftSideOfLine;
   private final YoDouble areaRatioThreshold;

   private final OccupancyGrid occupancyGrid;
   private final OccupancyGridVisualizer visualizer;

   private final FramePoint2D cellCenter = new FramePoint2D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public FootCoPHullCropper(String namePrefix,
                             ReferenceFrame soleFrame,
                             int nLengthSubdivisions,
                             int nWidthSubdivisions,
                             WalkingControllerParameters walkingControllerParameters,
                             ExplorationParameters explorationParameters,
                             YoGraphicsListRegistry yoGraphicsListRegistry,
                             YoVariableRegistry parentRegistry)
   {
      this.footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      this.footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      this.soleFrame = soleFrame;

      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);

      this.occupancyGrid = new OccupancyGrid(namePrefix, soleFrame, registry);

      this.nLengthSubdivisions = new YoInteger(namePrefix + "NLengthSubdivisions", registry);
      this.nLengthSubdivisions.set(nLengthSubdivisions);
      this.nWidthSubdivisions = new YoInteger(namePrefix + "NWidthSubdivisions", registry);
      this.nWidthSubdivisions.set(nWidthSubdivisions);

      occupancyGrid.setThresholdForCellOccupancy(defaultThresholdForCellActivation);
      occupancyGrid.setOccupancyDecayRate(defaultDecayRate);

      areaRatioThreshold = new YoDouble("areaRatioThreshold", registry);
      areaRatioThreshold.set(defaultAreaRatioThreshold);

      areaOnRightSideOfLine = new YoDouble(namePrefix + "AreaOnRightSideOfLine", registry);
      areaOnLeftSideOfLine = new YoDouble(namePrefix + "AreaOnLeftSideOfLine", registry);

      setupChangedGridParameterListeners();

      if (yoGraphicsListRegistry != null)
         visualizer = new OccupancyGridVisualizer(namePrefix, occupancyGrid, 100, 100, registry, yoGraphicsListRegistry);
      else
         visualizer = null;

      parentRegistry.addChild(registry);
   }

   private void setupChangedGridParameterListeners()
   {
      VariableChangedListener changedGridSizeListener = (v) ->
      {
         occupancyGrid.setCellXSize(footLength / nLengthSubdivisions.getIntegerValue());
         occupancyGrid.setCellYSize(footWidth / nWidthSubdivisions.getIntegerValue());
      };
      nLengthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      nWidthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      changedGridSizeListener.notifyOfVariableChange(null);
   }

   public void registerCenterOfPressureLocation(FramePoint2DReadOnly copToRegister)
   {
      occupancyGrid.registerPoint(copToRegister);
   }

   private FrameConvexPolygon2D convexHullOfCoPs = new FrameConvexPolygon2D();
   private FrameConvexPolygon2D leftSideCut = new FrameConvexPolygon2D();
   private FrameConvexPolygon2D rightSideCut = new FrameConvexPolygon2D();

   public RobotSide computeSideOfFootholdToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      computeConvexHull(convexHullOfCoPs);
      leftSideCut.setIncludingFrame(convexHullOfCoPs);
      rightSideCut.setIncludingFrame(convexHullOfCoPs);

      convexPolygonTools.cutPolygonWithLine(lineOfRotation, leftSideCut, RobotSide.LEFT);
      convexPolygonTools.cutPolygonWithLine(lineOfRotation, rightSideCut, RobotSide.RIGHT);

      areaOnLeftSideOfLine.set(rightSideCut.getArea());
      areaOnRightSideOfLine.set(leftSideCut.getArea());

      if (areaOnLeftSideOfLine.getDoubleValue() / areaOnRightSideOfLine.getDoubleValue() > areaRatioThreshold.getDoubleValue())
         return RobotSide.RIGHT;
      else if (areaOnRightSideOfLine.getDoubleValue() / areaOnLeftSideOfLine.getDoubleValue() > areaRatioThreshold.getDoubleValue())
         return RobotSide.LEFT;
      else
         return null;
   }

   public void computeConvexHull(FrameConvexPolygon2D convexHullToPack)
   {
      convexHullToPack.clear(soleFrame);

      List<OccupancyGridCell> activeCells = occupancyGrid.getAllActiveCells();
      for (int i = 0; i < activeCells.size(); i++)
      {
         OccupancyGridCell cell = activeCells.get(i);
         cellCenter.setIncludingFrame(soleFrame, occupancyGrid.getXLocation(cell.getXIndex()), occupancyGrid.getYLocation(cell.getYIndex()));
         if (cell.getIsOccupied())
            convexHullToPack.addVertex(cellCenter);
      }

      convexHullToPack.update();
   }

   public void reset()
   {
      occupancyGrid.reset();
      if (visualizer != null)
         visualizer.update();
   }

   public void update()
   {
      occupancyGrid.update();
      if (visualizer != null)
         visualizer.update();
   }
}
