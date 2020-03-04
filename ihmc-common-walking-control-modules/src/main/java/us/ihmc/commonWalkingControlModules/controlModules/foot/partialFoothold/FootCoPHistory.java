package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootCoPHistory
{
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 1.0;

   private final YoInteger nLengthSubdivisions;
   private final YoInteger nWidthSubdivisions;
   private final YoDouble thresholdForCellActivation;

   private final YoInteger currentXIndex;
   private final YoInteger currentYIndex;
   private final YoBoolean areCurrentCoPIndicesValid;

   private final YoFrameVector2D cellSize;
   private final YoDouble cellArea;

   private final ReferenceFrame soleFrame;
   private final Point2D tempPoint = new Point2D();
   private final FramePoint2D gridOrigin = new FramePoint2D();

   private final double footLength;
   private final double footWidth;
   private final DenseMatrix64F counterGrid = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F occupancyGrid = new DenseMatrix64F(1, 1);

   private final YoDouble decayRate;
   private final YoBoolean resetGridToEmpty;


   private final FrameLine2D shiftedLine = new FrameLine2D();
   private final FrameVector2D shiftingVector = new FrameVector2D();
   private final FramePoint2D cellCenter = new FramePoint2D();

   public FootCoPHistory(String namePrefix, ReferenceFrame soleFrame, int nLengthSubdivisions, int nWidthSubdivisions,
                         WalkingControllerParameters walkingControllerParameters, ExplorationParameters explorationParameters,
                         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      this.footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      this.soleFrame = soleFrame;
      gridOrigin.setIncludingFrame(soleFrame, -footLength, -footWidth);
      gridOrigin.scale(0.5);

      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);

      this.nLengthSubdivisions = new YoInteger(namePrefix + "NLengthSubdivisions", registry);
      this.nLengthSubdivisions.set(nLengthSubdivisions);
      this.nWidthSubdivisions = new YoInteger(namePrefix + "NWidthSubdivisions", registry);
      this.nWidthSubdivisions.set(nWidthSubdivisions);

      if (explorationParameters != null)
      {
         thresholdForCellActivation = explorationParameters.getCopGridThresholdForOccupancy();
         decayRate = explorationParameters.getCopGridDecayAlpha();
      }
      else
      {
         thresholdForCellActivation = new YoDouble(namePrefix + "ThresholdForCellActivation", registry);
         thresholdForCellActivation.set(defaultThresholdForCellActivation);
         decayRate = new YoDouble(namePrefix + "DecayRate", registry);
         decayRate.set(defaultDecayRate);
      }

      resetGridToEmpty = new YoBoolean(namePrefix + name + "Reset", registry);
      resetGridToEmpty.set(false);

      currentXIndex = new YoInteger(namePrefix + "CurrentXIndex", registry);
      currentYIndex = new YoInteger(namePrefix + "CurrentYIndex", registry);
      areCurrentCoPIndicesValid = new YoBoolean(namePrefix + "IsCurrentCoPIndicesValid", registry);

      cellSize = new YoFrameVector2D(namePrefix + "CellSize", soleFrame, registry);
      cellArea = new YoDouble(namePrefix + "CellArea", registry);

      setupChangedGridParameterListeners();

      parentRegistry.addChild(registry);
   }

   private void setupChangedGridParameterListeners()
   {
      VariableChangedListener changedGridSizeListener = (v) -> {
         counterGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
         occupancyGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
         cellSize.setX(footLength / nLengthSubdivisions.getIntegerValue());
         cellSize.setY(footWidth / nWidthSubdivisions.getIntegerValue());
         cellArea.set(cellSize.getX() * cellSize.getY());
      };
      nLengthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      nWidthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      changedGridSizeListener.notifyOfVariableChange(null);

      VariableChangedListener changedThresholdForCellActivationListener = (v) -> {

         for (int i = 0; i < nLengthSubdivisions.getIntegerValue(); i++)
         {
            for (int j = 0; j < nWidthSubdivisions.getIntegerValue(); j++)
            {
               if (counterGrid.get(i, j) >= thresholdForCellActivation.getDoubleValue())
                  occupancyGrid.set(i, j, 1.0);
               else
                  occupancyGrid.set(i, j, 0.0);
            }
         }
      };
      thresholdForCellActivation.addVariableChangedListener(changedThresholdForCellActivationListener);
      changedThresholdForCellActivationListener.notifyOfVariableChange(null);

      VariableChangedListener resetGridListener = (v) -> {

         if (resetGridToEmpty.getBooleanValue())
         {
            reset();
         }
         resetGridToEmpty.set(false);
      };
      resetGridToEmpty.addVariableChangedListener(resetGridListener);
      resetGridListener.notifyOfVariableChange(null);
   }

   public void registerCenterOfPressureLocation(FramePoint2DReadOnly copToRegister)
   {
      copToRegister.checkReferenceFrameMatch(soleFrame);
      tempPoint.sub(copToRegister, gridOrigin);

      int xIndex = findXIndex(tempPoint.getX());
      int yIndex = findYIndex(tempPoint.getY());

      currentXIndex.set(xIndex);
      currentYIndex.set(yIndex);

      areCurrentCoPIndicesValid.set(checkIfIndicesAreValid(xIndex, yIndex));

      if (areCurrentCoPIndicesValid.getBooleanValue())
      {
         counterGrid.add(xIndex, yIndex, 1.0);

         if (counterGrid.get(xIndex, yIndex) >= thresholdForCellActivation.getDoubleValue())
         {
            occupancyGrid.set(xIndex, yIndex, 1.0);
         }
         else
         {
            occupancyGrid.set(xIndex, yIndex, 0.0);
         }
      }
   }

   private boolean checkIfIndicesAreValid(int xIndex, int yIndex)
   {
      boolean isXIndexValid = xIndex < nLengthSubdivisions.getIntegerValue() && xIndex >= 0;
      boolean isYIndexValid = yIndex < nWidthSubdivisions.getIntegerValue() && yIndex >= 0;
      return isXIndexValid && isYIndexValid;
   }

   private int findXIndex(double x)
   {
      return (int) Math.floor(x / cellSize.getX());
   }

   private int findYIndex(double y)
   {
      return (int) Math.floor(y / cellSize.getY());
   }

   public boolean isCellOccupied(int xIndex, int yIndex)
   {
      return occupancyGrid.get(xIndex, yIndex) > 0.9;
   }

   public void getCellCenter(FramePoint2DBasics cellCenter, int xIndex, int yIndex)
   {
      double x = getXCoordinateInSoleFrame(xIndex);
      double y = getYCoordinateInSoleFrame(yIndex);

      cellCenter.setIncludingFrame(soleFrame, x, y);
   }

   private double getXCoordinateInSoleFrame(int xIndex)
   {
      return getCoordinateInSoleFrame(xIndex, 0);
   }

   private double getYCoordinateInSoleFrame(int yIndex)
   {
      return getCoordinateInSoleFrame(yIndex, 1);
   }

   private double getCoordinateInSoleFrame(int index, int element)
   {
      return (index + 0.5) * cellSize.getElement(element) + gridOrigin.getElement(element);
   }


   public int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2DReadOnly frameLine, RobotSide sideToLookAt, double minDistanceFromLine)
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
      double distanceToMoveAwayFromLine = Math.max(minDistanceFromLine, Math.abs(cellSize.getX() * Math.cos(theta) + cellSize.getY() * Math.sin(theta)));
      shiftingVector.scale(distanceToMoveAwayFromLine);

      // The point of the shiftedLine is shifted using the shiftingVector.
      shiftedLine.getPoint().add(shiftingVector);

      int numberOfCellsActivatedOnSideToLookAt = 0;
      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            getCellCenter(cellCenter, xIndex, yIndex);
            if (shiftedLine.isPointOnSideOfLine(cellCenter, sideToLookAt == RobotSide.LEFT))
               numberOfCellsActivatedOnSideToLookAt += occupancyGrid.get(xIndex, yIndex);
         }
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }

   public void reset()
   {
      counterGrid.zero();
      occupancyGrid.zero();
   }

   public void update()
   {
      double decay = decayRate.getDoubleValue();
      if (decay == 1.0)
         return;

      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            double value = counterGrid.get(xIndex, yIndex);
            counterGrid.set(xIndex, yIndex, value * decay);

            if (value >= thresholdForCellActivation.getDoubleValue())
            {
               occupancyGrid.set(xIndex, yIndex, 1.0);
            }
            else
            {
               occupancyGrid.set(xIndex, yIndex, 0.0);
            }
         }
      }
   }
}
