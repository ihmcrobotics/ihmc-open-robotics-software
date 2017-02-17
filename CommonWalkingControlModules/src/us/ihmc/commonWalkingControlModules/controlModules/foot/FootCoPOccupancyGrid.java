package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootCoPOccupancyGrid
{
   private static final boolean VISUALIZE = false;
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 1.0;

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry;

   private final IntegerYoVariable nLengthSubdivisions;
   private final IntegerYoVariable nWidthSubdivisions;
   private final DoubleYoVariable thresholdForCellActivation;

   private final IntegerYoVariable currentXIndex;
   private final IntegerYoVariable currentYIndex;
   private final BooleanYoVariable areCurrentCoPIndicesValid;

   private final YoFramePoint[][] cellViz;

   private final YoFrameVector2d cellSize;
   private final DoubleYoVariable cellArea;

   private final ReferenceFrame soleFrame;
   private final Point2D tempPoint = new Point2D();
   private final FramePoint2d gridOrigin = new FramePoint2d();
   private final FrameConvexPolygon2d gridBoundaries = new FrameConvexPolygon2d();

   private final double footLength;
   private final double footWidth;
   private final DenseMatrix64F counterGrid = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F occupancyGrid = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable decayRate;
   private final BooleanYoVariable resetGridToEmpty;

   public FootCoPOccupancyGrid(String namePrefix, ReferenceFrame soleFrame, int nLengthSubdivisions,
         int nWidthSubdivisions, WalkingControllerParameters walkingControllerParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footLength = walkingControllerParameters.getFootLength();
      this.footWidth = walkingControllerParameters.getFootWidth();
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
      this.nWidthSubdivisions.set(nWidthSubdivisions);

      ExplorationParameters explorationParameters = walkingControllerParameters.getOrCreateExplorationParameters(registry);
      if (explorationParameters != null)
      {
         thresholdForCellActivation = explorationParameters.getCopGridThresholdForOccupancy();
         decayRate = explorationParameters.getCopGridDecayAlpha();
      }
      else
      {
         thresholdForCellActivation = new DoubleYoVariable(namePrefix + "ThresholdForCellActivation", registry);
         thresholdForCellActivation.set(defaultThresholdForCellActivation);
         decayRate = new DoubleYoVariable(namePrefix + "DecayRate", registry);
         decayRate.set(defaultDecayRate);
      }

      resetGridToEmpty = new BooleanYoVariable(namePrefix + name + "Reset", registry);
      resetGridToEmpty.set(false);

      currentXIndex = new IntegerYoVariable(namePrefix + "CurrentXIndex", registry);
      currentYIndex = new IntegerYoVariable(namePrefix + "CurrentYIndex", registry);
      areCurrentCoPIndicesValid = new BooleanYoVariable(namePrefix + "IsCurrentCoPIndicesValid", registry);

      cellSize = new YoFrameVector2d(namePrefix + "CellSize", soleFrame, registry);
      cellArea = new DoubleYoVariable(namePrefix + "CellArea", registry);

      setupChangedGridParameterListeners();

      if (VISUALIZE)
      {
         cellViz = new YoFramePoint[nLengthSubdivisions][nWidthSubdivisions];
         for (int i = 0; i < cellViz.length; i++)
         {
            for (int j = 0; j < cellViz[0].length; j++)
            {
               String namePrefix2 = "CellViz_X" + String.valueOf(i) + "Y" + String.valueOf(j);
               YoFramePoint pointForViz = new YoFramePoint(namePrefix + namePrefix2, ReferenceFrame.getWorldFrame(), registry);
               pointForViz.setToNaN();
               cellViz[i][j] = pointForViz;
               YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(namePrefix + namePrefix2, pointForViz, 0.004, YoAppearance.Orange());
               yoGraphicsListRegistry.registerArtifact(name, yoGraphicPosition.createArtifact());
               yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicPosition);
            }
         }
      }
      else
      {
         cellViz = null;
      }

      parentRegistry.addChild(registry);
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
                  if (counterGrid.get(i, j) >= thresholdForCellActivation.getDoubleValue())
                     occupancyGrid.set(i, j, 1.0);
                  else
                     occupancyGrid.set(i, j, 0.0);
               }
            }
         }
      };
      thresholdForCellActivation.addVariableChangedListener(changedThresholdForCellActivationListener);
      changedThresholdForCellActivationListener.variableChanged(null);

      VariableChangedListener resetGridListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (resetGridToEmpty.getBooleanValue())
            {
               reset();
            }
            resetGridToEmpty.set(false);
         }
      };
      resetGridToEmpty.addVariableChangedListener(resetGridListener);
      resetGridListener.variableChanged(null);
   }

   private final FramePoint cellPosition = new FramePoint();

   public void registerCenterOfPressureLocation(FramePoint2d copToRegister)
   {
      copToRegister.checkReferenceFrameMatch(soleFrame);
      copToRegister.get(tempPoint);

      tempPoint.setX(tempPoint.getX() - gridOrigin.getX());
      tempPoint.setY(tempPoint.getY() - gridOrigin.getY());

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
      tempPoint.setX(tempPoint.getX() - gridOrigin.getX());
      tempPoint.setY(tempPoint.getY() - gridOrigin.getY());

      int xIndex = findXIndex(tempPoint.getX());
      int yIndex = findYIndex(tempPoint.getY());
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

   public void update()
   {
      double decay = decayRate.getDoubleValue();
      if (decay == 1.0) return;

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

            if (VISUALIZE)
            {
               if (isCellOccupied(xIndex, yIndex))
               {
                  getCellCenter(cellCenter, xIndex, yIndex);
                  cellPosition.setXYIncludingFrame(cellCenter);
                  cellViz[xIndex][yIndex].setAndMatchFrame(cellPosition);
               }
               else
               {
                  cellViz[xIndex][yIndex].setToNaN();
               }
            }
         }
      }
   }

   private final FramePoint2d tempCellCenter = new FramePoint2d();
   public void computeConvexHull(FrameConvexPolygon2d convexHullToPack)
   {
      convexHullToPack.clear(soleFrame);
      tempCellCenter.setToZero(soleFrame);

      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            if (isCellOccupied(xIndex, yIndex))
            {
               getCellCenter(tempCellCenter, xIndex, yIndex);
               convexHullToPack.addVertex(tempCellCenter);
            }
         }
      }

      convexHullToPack.update();
   }

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
   private final DenseMatrix64F pointCloud = new DenseMatrix64F(0, 0);
   private final Point3D tempPoint3d = new Point3D();
   private final FramePoint2d lineOrigin = new FramePoint2d();
   private final Vector3D tempVector3d = new Vector3D();
   private final FrameVector2d lineDirection = new FrameVector2d();

   private final FramePoint2d pointA = new FramePoint2d();
   private final FramePoint2d pointB = new FramePoint2d();

   public boolean fitLineToData(FrameLine2d lineToPack)
   {
      // TODO: instead of counting keep track of number of occupied positions
      int numberOfPoints = 0;
      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            if (isCellOccupied(xIndex, yIndex))
            {
               numberOfPoints++;
            }
         }
      }

      if (numberOfPoints < 2)
         return false;

      if (numberOfPoints == 2)
      {
         for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
         {
            for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
            {
               if (isCellOccupied(xIndex, yIndex))
               {
                  getCellCenter(tempCellCenter, xIndex, yIndex);
                  pointB.setIncludingFrame(pointA);
                  pointA.setIncludingFrame(tempCellCenter);
               }
            }
         }
         lineToPack.set(pointA, pointB);
         return true;
      }

      pointCloud.reshape(3, numberOfPoints);

      int counter = 0;
      for (int xIndex = 0; xIndex < nLengthSubdivisions.getIntegerValue(); xIndex++)
      {
         for (int yIndex = 0; yIndex < nWidthSubdivisions.getIntegerValue(); yIndex++)
         {
            if (isCellOccupied(xIndex, yIndex))
            {
               getCellCenter(tempCellCenter, xIndex, yIndex);
               pointCloud.set(0, counter, tempCellCenter.getX());
               pointCloud.set(1, counter, tempCellCenter.getY());
               // TODO: make 2D PCA class
               pointCloud.set(2, counter, 0.0);
               counter++;
            }
         }
      }

      pca.setPointCloud(pointCloud);
      pca.compute();
      pca.getMean(tempPoint3d);
      pca.getPrincipalVector(tempVector3d);

      lineOrigin.setIncludingFrame(soleFrame, tempPoint3d.getX(), tempPoint3d.getY());
      lineDirection.setIncludingFrame(soleFrame, tempVector3d.getX(), tempVector3d.getY());

      if (lineDirection.containsNaN())
         return false;

      lineToPack.setToZero(soleFrame);
      lineToPack.setPoint(lineOrigin);
      lineToPack.setVector(lineDirection);
      return true;
   }
}
