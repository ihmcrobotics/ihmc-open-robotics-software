package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootCoPHullCalculator
{
   private final YoDouble areaOnRightSideOfLine;
   private final YoDouble areaOnLeftSideOfLine;
   private final DoubleProvider areaRatioThreshold;

   private final OccupancyGrid occupancyGrid;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final YoEnum<RobotSide> sideOfFootToCrop;

   public FootCoPHullCalculator(String namePrefix,
                                OccupancyGrid occupancyGrid,
                                FootholdRotationParameters rotationParameters,
                                YoRegistry parentRegistry)
   {
      this.occupancyGrid = occupancyGrid;
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(namePrefix + name);

      sideOfFootToCrop = new YoEnum<>(namePrefix + "HullSideOfFootToCrop", registry, RobotSide.class, true);

      areaRatioThreshold = rotationParameters.getCopHullAreaRatioThreshold();

      areaOnRightSideOfLine = new YoDouble(namePrefix + "AreaOnRightSideOfLine", registry);
      areaOnLeftSideOfLine = new YoDouble(namePrefix + "AreaOnLeftSideOfLine", registry);

      reset();

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

      if (areaOnLeftSideOfLine.getDoubleValue() / areaOnRightSideOfLine.getDoubleValue() > areaRatioThreshold.getValue())
         sideOfFootToCrop.set(RobotSide.RIGHT);
      else if (areaOnRightSideOfLine.getDoubleValue() / areaOnLeftSideOfLine.getDoubleValue() > areaRatioThreshold.getValue())
         sideOfFootToCrop.set(RobotSide.LEFT);
      else
         sideOfFootToCrop.set(null);

      return sideOfFootToCrop.getEnumValue();
   }

   public void reset()
   {
      sideOfFootToCrop.set(null);
      areaOnLeftSideOfLine.setToNaN();
      areaOnRightSideOfLine.setToNaN();
   }
}
