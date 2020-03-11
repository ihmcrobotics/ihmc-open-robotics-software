package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridCell;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class CropVerifier
{
   private final OccupancyGrid occupancyGrid;
   private final YoDouble perpendicularCoPError;

   private final IntegerProvider numberOfCellsThreshold;
   private final DoubleProvider perpendicularCopErrorThreshold;
   private final YoInteger numberOfCellsOccupiedOnCropSide;
   private final YoBoolean desiredCopOnCorrectSide;
   private final YoBoolean perpendicularCopErrorAboveThreshold;
   private final YoBoolean enoughDesiredCopOnCropSide;

   private final FrameLine2D shiftedLine = new FrameLine2D();
   private final FrameVector2D shiftingVector = new FrameVector2D();
   private final FramePoint2D cellCenter = new FramePoint2D();
   private final ReferenceFrame soleFrame;

   public CropVerifier(String namePrefix,
                       ReferenceFrame soleFrame,
                       double lengthResolution,
                       double widthResoultion,
                       FootholdRotationParameters explorationParameters,
                       YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;

      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.occupancyGrid = new OccupancyGrid(namePrefix, soleFrame, registry);
      occupancyGrid.setCellXSize(lengthResolution);
      occupancyGrid.setCellYSize(widthResoultion);

      perpendicularCoPError = new YoDouble(namePrefix + "PerpendicularCopError", registry);
      perpendicularCopErrorThreshold = explorationParameters.getPerpendicularCoPErrorThreshold();
      perpendicularCopErrorAboveThreshold = new YoBoolean(namePrefix + "PerpendicularCopErrorAboveThreshold", registry);
      enoughDesiredCopOnCropSide = new YoBoolean(namePrefix + "EnoughDesiredCopOnCropSide", registry);
      numberOfCellsThreshold = explorationParameters.getNumberOfDesiredCopsOnCropSide();

      numberOfCellsOccupiedOnCropSide = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnCropSide", registry);

      desiredCopOnCorrectSide = new YoBoolean(namePrefix + "DesiredCopOnCorrectSide", registry);


      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      occupancyGrid.reset();
   }

   public void update(FramePoint2DReadOnly desiredCoP)
   {
      occupancyGrid.update();
      occupancyGrid.registerPoint(desiredCoP);
   }

   public boolean verifyFootholdCrop(FramePoint2DReadOnly desiredCoP, RobotSide sideToCrop, FrameLine2DReadOnly lineOfRotation)
   {
      perpendicularCoPError.set(lineOfRotation.distance(desiredCoP));
      perpendicularCopErrorAboveThreshold.set(perpendicularCoPError.getDoubleValue() > perpendicularCopErrorThreshold.getValue());

      desiredCopOnCorrectSide.set(lineOfRotation.isPointOnSideOfLine(desiredCoP, sideToCrop == RobotSide.LEFT));
      numberOfCellsOccupiedOnCropSide.set(computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation, sideToCrop, 0.0));
      enoughDesiredCopOnCropSide.set(numberOfCellsOccupiedOnCropSide.getValue() > numberOfCellsThreshold.getValue());

      return perpendicularCopErrorAboveThreshold.getBooleanValue() && desiredCopOnCorrectSide.getBooleanValue() && enoughDesiredCopOnCropSide.getBooleanValue();
   }

   private int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2DReadOnly frameLine, RobotSide sideToLookAt, double minDistanceFromLine)
   {
      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.
      frameLine.checkReferenceFrameMatch(soleFrame);
      shiftingVector.setIncludingFrame(frameLine.getDirection());
      shiftedLine.setIncludingFrame(frameLine);

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      EuclidGeometryTools.perpendicularVector2D(shiftingVector);
      if (sideToLookAt == RobotSide.RIGHT)
      {
         shiftingVector.negate();
      }

      double theta = Math.atan2(shiftedLine.getDirection().getY(), shiftedLine.getDirection().getX());

      // It is scaled such that the line is being shifted by one cell or minDistanceFromLine depending on which one is the greatest.
      double cellXSize = occupancyGrid.getCellXSize();
      double cellYSize = occupancyGrid.getCellYSize();
      double distanceToMoveAwayFromLine = Math.max(minDistanceFromLine, Math.abs(cellXSize * Math.cos(theta) + cellYSize * Math.sin(theta)));
      shiftingVector.scale(distanceToMoveAwayFromLine);

      // The point of the shiftedLine is shifted using the shiftingVector.
      shiftedLine.getPoint().add(shiftingVector);

      int numberOfCellsActivatedOnSideToLookAt = 0;
      List<OccupancyGridCell> activeCells = occupancyGrid.getAllActiveCells();
      for (int i = 0; i < activeCells.size(); i++)
      {
         OccupancyGridCell cell = activeCells.get(i);
         cellCenter.setIncludingFrame(soleFrame, occupancyGrid.getXLocation(cell.getXIndex()), occupancyGrid.getYLocation(cell.getYIndex()));
         if (shiftedLine.isPointOnSideOfLine(cellCenter, sideToLookAt == RobotSide.LEFT) && cell.getIsOccupied())
            numberOfCellsActivatedOnSideToLookAt++;
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }


}
