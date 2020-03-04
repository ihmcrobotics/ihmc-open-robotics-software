package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootCoPOccupancyGrid;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CroppedFootholdCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FrameConvexPolygon2D defaultFootPolygon;
   private final FrameConvexPolygon2D shrunkenFootPolygon;

   private final YoInteger numberOfCellsOccupiedOnRightSideOfLine;
   private final YoInteger numberOfCellsOccupiedOnLeftSideOfLine;
   private final SideDependentList<YoInteger> numberOfOccupiedCells;
   private final YoInteger thresholdForCoPRegionOccupancy;
   private final YoDouble distanceFromLineOfRotationToComputeCoPOccupancy;

   private final YoDouble minAreaToConsider;
   private final YoBoolean hasEnoughAreaToCrop;

   private final FootCoPHistory footCoPOccupancyGrid;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public CroppedFootholdCalculator(String namePrefix, ReferenceFrame soleFrame, ContactableFoot contactableFoot,
                                    WalkingControllerParameters walkingControllerParameters,
                                    ExplorationParameters explorationParameters, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
      shrunkenFootPolygon = new FrameConvexPolygon2D(defaultFootPolygon);

      footCoPOccupancyGrid = new FootCoPHistory(namePrefix, soleFrame, 40, 20, walkingControllerParameters, explorationParameters, yoGraphicsListRegistry,
                                                      registry);

      numberOfCellsOccupiedOnRightSideOfLine = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnRightSideOfLine", registry);
      numberOfCellsOccupiedOnLeftSideOfLine = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnLeftSideOfLine", registry);
      hasEnoughAreaToCrop = new YoBoolean(namePrefix + "HasEnoughAreaToCrop", registry);

      numberOfOccupiedCells = new SideDependentList<>(numberOfCellsOccupiedOnLeftSideOfLine, numberOfCellsOccupiedOnRightSideOfLine);

      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();
      minAreaToConsider = explorationParameters.getMinAreaToConsider();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      reset(defaultFootPolygon);
   }

   public void reset(FrameConvexPolygon2DReadOnly polygon)
   {
      shrunkenFootPolygon.set(polygon);
   }

   public void update(FramePoint2DReadOnly measuredCoP)
   {
      footCoPOccupancyGrid.update();
      if (measuredCoP.containsNaN())
         return;

      footCoPOccupancyGrid.registerCenterOfPressureLocation(measuredCoP);
   }

   public void computeShrunkFoothold(FrameLine2DReadOnly lineOfRotation)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         numberOfOccupiedCells.get(robotSide).set(footCoPOccupancyGrid.computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation, robotSide,
                                                                                                             distanceFromLineOfRotationToComputeCoPOccupancy
                                                                                                                   .getDoubleValue()));
      }

      boolean leftOccupied = numberOfOccupiedCells.get(RobotSide.LEFT).getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();
      boolean rightOccupied = numberOfOccupiedCells.get(RobotSide.RIGHT).getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();

      if (leftOccupied && rightOccupied)
         throw new RuntimeException("Error: both can't be occupied.");

      hasEnoughAreaToCrop.set(shrunkenFootPolygon.getArea() > minAreaToConsider.getDoubleValue());

      if ((leftOccupied || rightOccupied) && hasEnoughAreaToCrop.getBooleanValue())
      {
         RobotSide sideToCrop = leftOccupied ? RobotSide.RIGHT : RobotSide.LEFT;
         convexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkenFootPolygon, sideToCrop);
      }
   }
}
