package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Point2d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class FootCoPOccupancyGrid
{
   private static final boolean VISUALIZE = false;

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry;

   private final IntegerYoVariable nLengthSubdivisions;
   private final IntegerYoVariable nWidthSubdivisions;
   private final IntegerYoVariable thresholdForCellActivation;

   private final IntegerYoVariable currentXIndex;
   private final IntegerYoVariable currentYIndex;
   private final BooleanYoVariable areCurrentCoPIndicesValid;

   private final YoFramePoint[][] cellViz;

   private final YoFrameVector2d cellSize;
   private final DoubleYoVariable cellArea;

   private final ReferenceFrame soleFrame;
   private final Point2d tempPoint = new Point2d();
   private final FramePoint2d gridOrigin = new FramePoint2d();
   private final FrameConvexPolygon2d gridBoundaries = new FrameConvexPolygon2d();

   private final double footLength;
   private final double footWidth;
   private final DenseMatrix64F counterGrid = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F occupancyGrid = new DenseMatrix64F(1, 1);

   public FootCoPOccupancyGrid(String namePrefix, ReferenceFrame soleFrame, double footLength, double footWidth, int nLengthSubdivisions,
         int nWidthSubdivisions, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footLength = footLength;
      this.footWidth = footWidth;
      this.soleFrame = soleFrame;
      gridOrigin.setIncludingFrame(soleFrame, -footLength, -footWidth);
      gridOrigin.scale(0.5);

      gridBoundaries.clear(soleFrame);
      gridBoundaries.addVertex(new FramePoint2d(soleFrame, -footLength / 2.0, -footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2d(soleFrame, -footLength / 2.0, footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2d(soleFrame, footLength / 2.0, -footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2d(soleFrame, footLength / 2.0, footWidth / 2.0));
      gridBoundaries.update();

      registry = new YoVariableRegistry(namePrefix + name);

      this.nLengthSubdivisions = new IntegerYoVariable(namePrefix + "NLengthSubdivisions", registry);
      this.nLengthSubdivisions.set(nLengthSubdivisions);
      this.nWidthSubdivisions = new IntegerYoVariable(namePrefix + "NWidthSubdivisions", registry);
      this.nWidthSubdivisions.set(nLengthSubdivisions);
      thresholdForCellActivation = new IntegerYoVariable(namePrefix + "ThresholdForCellActivation", registry);
      thresholdForCellActivation.set(1);

      currentXIndex = new IntegerYoVariable(namePrefix + "CurrentXIndex", registry);
      currentYIndex = new IntegerYoVariable(namePrefix + "CurrentYIndex", registry);
      areCurrentCoPIndicesValid = new BooleanYoVariable(namePrefix + "IsCurrentCoPIndicesValid", registry);

      cellSize = new YoFrameVector2d(namePrefix + "CellSize", soleFrame, registry);
      cellArea = new DoubleYoVariable(namePrefix + "CellArea", registry);

      setupChangedGridParameterListeners();

      if (VISUALIZE)
      {
         cellViz = new YoFramePoint[20][20];
         for (int i = 0; i < cellViz.length; i++)
         {
            for (int j = 0; j < cellViz[0].length; j++)
            {
               String namePrefix2 = "CellViz_X" + String.valueOf(i) + "Y" + String.valueOf(j);
               YoFramePoint pointForViz = new YoFramePoint(namePrefix + namePrefix2, ReferenceFrame.getWorldFrame(), registry);
               pointForViz.setToNaN();
               cellViz[i][j] = pointForViz;
               YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(namePrefix + namePrefix2, pointForViz, 0.008, YoAppearance.Black());
               yoGraphicsListRegistry.registerArtifact(name, yoGraphicPosition.createArtifact());
               yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicPosition);
            }
         }
      }
      else
      {
         cellViz = null;
      }
   }

   private void setupChangedGridParameterListeners()
   {
      VariableChangedListener changedGridSizeListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            counterGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
            occupancyGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
            cellSize.setX(footLength / nLengthSubdivisions.getIntegerValue());
            cellSize.setY(footWidth / nWidthSubdivisions.getIntegerValue());
            cellArea.set(cellSize.getX() * cellSize.getY());
         }
      };

      nLengthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      nWidthSubdivisions.addVariableChangedListener(changedGridSizeListener);
      changedGridSizeListener.variableChanged(null);

      VariableChangedListener changedThresholdForCellActivationListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (int i = 0; i < nLengthSubdivisions.getIntegerValue(); i++)
            {
               for (int j = 0; j < nWidthSubdivisions.getIntegerValue(); j++)
               {
                  if (counterGrid.get(i, j) >= thresholdForCellActivation.getIntegerValue())
                     occupancyGrid.set(i, j, 1.0);
                  else
                     occupancyGrid.set(i, j, 0.0);
               }
            }
         }
      };

      thresholdForCellActivation.addVariableChangedListener(changedThresholdForCellActivationListener);
      changedThresholdForCellActivationListener.variableChanged(null);
   }

   public void setThresholdForCellActivation(int newThreshold)
   {
      thresholdForCellActivation.set(newThreshold);
   }

   private final FramePoint cellPosition = new FramePoint();

   public void registerCenterOfPressureLocation(FramePoint2d copToRegister)
   {
      copToRegister.checkReferenceFrameMatch(soleFrame);
      copToRegister.get(tempPoint);

      tempPoint.x -= gridOrigin.getX();
      tempPoint.y -= gridOrigin.getY();

      int xIndex = findXIndex(tempPoint.x);
      int yIndex = findYIndex(tempPoint.y);

      currentXIndex.set(xIndex);
      currentYIndex.set(yIndex);

      areCurrentCoPIndicesValid.set(checkIfIndicesAreValid(xIndex, yIndex));

      if (areCurrentCoPIndicesValid.getBooleanValue())
      {
         counterGrid.add(xIndex, yIndex, 1.0);

         if (counterGrid.get(xIndex, yIndex) >= thresholdForCellActivation.getIntegerValue())
         {
            occupancyGrid.set(xIndex, yIndex, 1.0);
            if (VISUALIZE)
            {
               getCellCenter(cellCenter, xIndex, yIndex);
               cellPosition.setXYIncludingFrame(cellCenter);
               cellViz[xIndex][yIndex].setAndMatchFrame(cellPosition);
            }

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

   public boolean isCellAtLocationOccupied(FramePoint2d location)
   {
      location.checkReferenceFrameMatch(soleFrame);
      location.get(tempPoint);
      tempPoint.x -= gridOrigin.getX();
      tempPoint.y -= gridOrigin.getY();

      int xIndex = findXIndex(tempPoint.x);
      int yIndex = findYIndex(tempPoint.y);
      if (checkIfIndicesAreValid(xIndex, yIndex))
         return occupancyGrid.get(xIndex, yIndex) > 0.9;
      else
         return false;
   }

   public void getCellCenter(FramePoint2d cellCenter, int xIndex, int yIndex)
   {
      double x = getXCoordinateInSoleFrame(xIndex);
      double y = getYCoordinateInSoleFrame(yIndex);

      cellCenter.setIncludingFrame(soleFrame, x, y);
   }

   private double getXCoordinateInSoleFrame(int xIndex)
   {
      return ((double) xIndex + 0.5) * cellSize.getX() + gridOrigin.getX();
   }

   private double getYCoordinateInSoleFrame(int yIndex)
   {
      return ((double) yIndex + 0.5) * cellSize.getY() + gridOrigin.getY();
   }

   public boolean findCenterOfClosestCell(FramePoint2d centerOfClosestCellToPack, FramePoint2d closestToPoint)
   {
      int xIndex = findXIndex(closestToPoint.getX());
      int yIndex = findYIndex(closestToPoint.getY());

      if (!checkIfIndicesAreValid(xIndex, yIndex))
         return false;

      getCellCenter(centerOfClosestCellToPack, xIndex, yIndex);
      return true;
   }

   private final FrameLine2d shiftedLine = new FrameLine2d();
   private final FrameVector2d shiftedLineVector = new FrameVector2d();
   private final FramePoint2d shiftedLinePoint = new FramePoint2d();
   private final FrameVector2d shiftingVector = new FrameVector2d();
   private final FramePoint2d cellCenter = new FramePoint2d();

   /**
    * This algorithm is stupid because it checks for every cell if the cell is on the right side and if it is activated.
    * @param frameLine
    * @param sideToLookAt
    * @return
    */
   public int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2d frameLine, RobotSide sideToLookAt)
   {
      return computeNumberOfCellsOccupiedOnSideOfLine(frameLine, sideToLookAt, 0.0);
   }

   public int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2d frameLine, RobotSide sideToLookAt, double minDistanceFromLine)
   {
      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.
      frameLine.checkReferenceFrameMatch(soleFrame);
      frameLine.getNormalizedFrameVector(shiftingVector);

      frameLine.getFramePoint2d(shiftedLinePoint);
      frameLine.getNormalizedFrameVector(shiftedLineVector);

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      shiftingVector.rotate90();
      if (sideToLookAt == RobotSide.RIGHT)
      {
         shiftingVector.negate();
      }

      double theta = Math.atan2(shiftedLineVector.getY(), shiftedLineVector.getX());

      // It is scaled such that the line is being shifted by one cell or minDistanceFromLine depending on which one is the greatest.
      double distanceToMoveAwayFromLine = Math.max(minDistanceFromLine, Math.abs(cellSize.getX() * Math.cos(theta) + cellSize.getY() * Math.sin(theta)));
      shiftingVector.scale(distanceToMoveAwayFromLine);

      // The point of the shiftedLine is shifted using the shiftingVector.
      shiftedLinePoint.add(shiftingVector);
      // Setup here the shiftedLined that is used from now on.
      shiftedLine.setIncludingFrame(shiftedLinePoint, shiftedLineVector);

      int numberOfCellsActivatedOnSideToLookAt = 0;
      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            getCellCenter(cellCenter, xIndex, yIndex);
            if (shiftedLine.isPointOnSideOfLine(cellCenter, sideToLookAt))
               numberOfCellsActivatedOnSideToLookAt += occupancyGrid.get(xIndex, yIndex);
         }
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }

   /**
    * This algorithm is smarter since it doesn't go through all the cells, but it probably doesn't work :/
    * @param frameLine
    * @param sideToLookAt
    * @return
    */
   public int computeNumberOfCellsOccupiedOnSideOfLineSmarter(FrameLine2d frameLine, RobotSide sideToLookAt)
   {
      int returnFailure = -1;

      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.

      frameLine.checkReferenceFrameMatch(soleFrame);
      frameLine.getNormalizedFrameVector(shiftingVector);

      frameLine.getFramePoint2d(shiftedLinePoint);
      frameLine.getNormalizedFrameVector(shiftedLineVector);

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      shiftingVector.rotate90();
      if (sideToLookAt == RobotSide.RIGHT)
      {
         shiftingVector.negate();
      }

      double theta = Math.atan2(shiftedLineVector.getY(), shiftedLineVector.getX());

      // It is scaled such that the line is being shifted by one cell.
      shiftingVector.scale(cellSize.getX() * Math.cos(theta) + cellSize.getY() * Math.sin(theta));

      // The point of the shiftedLine is shifted using the shiftingVector.
      shiftedLinePoint.add(shiftingVector);
      // Setup here the shiftedLined that is used from now on.
      shiftedLine.setIncludingFrame(shiftedLinePoint, shiftedLineVector);

      // I tried to make it clever such that this algorithm doesn't check every of the cells

      // Get the intersections
      FramePoint2d[] intersections = gridBoundaries.intersectionWith(shiftedLine);

      if (intersections == null || intersections.length == 1)
         return returnFailure;

      FrameVector2d intersectionsVector = new FrameVector2d(soleFrame);
      intersectionsVector.sub(intersections[1], intersections[0]);

      FramePoint2d temp = new FramePoint2d(soleFrame);
      FramePoint2d cellCenter = new FramePoint2d();
      int xIndex = -1;
      int yIndex = -1;

      if (intersectionsVector.dot(shiftedLineVector) > 0.5)
      {
         temp.setIncludingFrame(intersections[0]);
      }
      else
      {
         temp.setIncludingFrame(intersections[1]);
      }

      getCellCenter(cellCenter, xIndex, yIndex);
      double xDirection = Math.signum(shiftingVector.getX());
      double yDirection = Math.signum(shiftingVector.getY());

      shiftedLineVector.normalize();
      if (Math.abs(theta % Math.PI / 2.0) > 0.1)
      {
         shiftedLineVector.scale(cellSize.getX() / Math.cos(theta));

         int cellActivatedNumber = 0;

         while (xIndex < nLengthSubdivisions.getIntegerValue())
         {
            while (yIndex < nWidthSubdivisions.getIntegerValue())
            {
               cellActivatedNumber += (int) occupancyGrid.get(xIndex, yIndex);
               yIndex += (int) yDirection;
            }

            temp.add(shiftedLineVector);
            xIndex = findXIndex(temp.getX());
            yIndex = findYIndex(temp.getY());
         }

         return cellActivatedNumber;
      }
      else
      {
         shiftedLineVector.scale(cellSize.getY() / Math.sin(theta));

         int cellActivatedNumber = 0;

         while (yIndex < nWidthSubdivisions.getIntegerValue())
         {
            while (xIndex < nLengthSubdivisions.getIntegerValue())
            {
               cellActivatedNumber += (int) occupancyGrid.get(xIndex, yIndex);
               xIndex += (int) xDirection;
            }

            temp.add(shiftedLineVector);
            xIndex = findXIndex(temp.getX());
            yIndex = findYIndex(temp.getY());
         }

         return cellActivatedNumber;
      }
   }

   public void reset()
   {
      counterGrid.zero();
      occupancyGrid.zero();

      if (VISUALIZE)
      {
         for (int i = 0; i < cellViz.length; i++)
         {
            for (int j = 0; j < cellViz[0].length; j++)
            {
               cellViz[i][j].setToNaN();
            }
         }
      }
   }
}
