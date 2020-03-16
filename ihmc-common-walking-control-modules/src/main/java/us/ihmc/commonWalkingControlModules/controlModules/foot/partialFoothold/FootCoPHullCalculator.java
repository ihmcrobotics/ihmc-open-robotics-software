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
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class FootCoPHullCalculator
{
   private static final double defaultAreaRatioThreshold = 0.6;

   private final YoDouble areaOnRightSideOfLine;
   private final YoDouble areaOnLeftSideOfLine;
   private final YoDouble areaRatioThreshold;

   private final OccupancyGrid occupancyGrid;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final YoEnum<RobotSide> sideOfFootToCrop;

   public FootCoPHullCalculator(String namePrefix,
                                OccupancyGrid occupancyGrid,
                                YoVariableRegistry parentRegistry)
   {
      this.occupancyGrid = occupancyGrid;
      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);

      sideOfFootToCrop = new YoEnum<>(namePrefix + "HullSideOfFootToCrop", registry, RobotSide.class, true);


      areaRatioThreshold = new YoDouble("areaRatioThreshold", registry);
      areaRatioThreshold.set(defaultAreaRatioThreshold);

      areaOnRightSideOfLine = new YoDouble(namePrefix + "AreaOnRightSideOfLine", registry);
      areaOnLeftSideOfLine = new YoDouble(namePrefix + "AreaOnLeftSideOfLine", registry);

      parentRegistry.addChild(registry);
   }

   private FrameConvexPolygon2D convexHullOfCoPs = new FrameConvexPolygon2D();
   private FrameConvexPolygon2D leftSideCut = new FrameConvexPolygon2D();
   private FrameConvexPolygon2D rightSideCut = new FrameConvexPolygon2D();

   public RobotSide computeSideOfFootholdToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, convexHullOfCoPs);

      leftSideCut.setIncludingFrame(convexHullOfCoPs);
      rightSideCut.setIncludingFrame(convexHullOfCoPs);

      convexPolygonTools.cutPolygonWithLine(lineOfRotation, leftSideCut, RobotSide.LEFT);
      convexPolygonTools.cutPolygonWithLine(lineOfRotation, rightSideCut, RobotSide.RIGHT);

      areaOnLeftSideOfLine.set(rightSideCut.getArea());
      areaOnRightSideOfLine.set(leftSideCut.getArea());

      if (areaOnLeftSideOfLine.getDoubleValue() / areaOnRightSideOfLine.getDoubleValue() > areaRatioThreshold.getDoubleValue())
         sideOfFootToCrop.set(RobotSide.RIGHT);
      else if (areaOnRightSideOfLine.getDoubleValue() / areaOnLeftSideOfLine.getDoubleValue() > areaRatioThreshold.getDoubleValue())
         sideOfFootToCrop.set(RobotSide.LEFT);
      else
         sideOfFootToCrop.set(null);

      return sideOfFootToCrop.getEnumValue();
   }

   public void reset()
   {
      sideOfFootToCrop.set(null);
   }
}
