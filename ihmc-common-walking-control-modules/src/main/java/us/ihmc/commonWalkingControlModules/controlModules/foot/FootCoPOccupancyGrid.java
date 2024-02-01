package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class FootCoPOccupancyGrid implements SCS2YoGraphicHolder
{
   private static final boolean VISUALIZE = false;
   private static final double defaultThresholdForCellActivation = 1.0;
   private static final double defaultDecayRate = 1.0;

   private final String name = getClass().getSimpleName();

   private final YoRegistry registry;

   private final YoInteger nLengthSubdivisions;
   private final YoInteger nWidthSubdivisions;
   private final YoDouble thresholdForCellActivation;

   private final YoInteger currentXIndex;
   private final YoInteger currentYIndex;
   private final YoBoolean areCurrentCoPIndicesValid;

   private final YoFramePoint3D[][] cellViz;

   private final YoFrameVector2D cellSize;
   private final YoDouble cellArea;

   private final ReferenceFrame soleFrame;
   private final Point2D tempPoint = new Point2D();
   private final FramePoint2D gridOrigin = new FramePoint2D();
   private final FrameConvexPolygon2D gridBoundaries = new FrameConvexPolygon2D();

   private final double footLength;
   private final double footWidth;
   private final DMatrixRMaj counterGrid = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj occupancyGrid = new DMatrixRMaj(1, 1);

   private final YoDouble decayRate;
   private final YoBoolean resetGridToEmpty;

   public FootCoPOccupancyGrid(String namePrefix,
                               ReferenceFrame soleFrame,
                               int nLengthSubdivisions,
                               int nWidthSubdivisions,
                               WalkingControllerParameters walkingControllerParameters,
                               ExplorationParameters explorationParameters,
                               YoGraphicsListRegistry yoGraphicsListRegistry,
                               YoRegistry parentRegistry)
   {
      this.footLength = walkingControllerParameters.getSteppingParameters().getFootLength();
      this.footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      this.soleFrame = soleFrame;
      gridOrigin.setIncludingFrame(soleFrame, -footLength, -footWidth);
      gridOrigin.scale(0.5);

      gridBoundaries.clear(soleFrame);
      gridBoundaries.addVertex(new FramePoint2D(soleFrame, -footLength / 2.0, -footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2D(soleFrame, -footLength / 2.0, footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2D(soleFrame, footLength / 2.0, -footWidth / 2.0));
      gridBoundaries.addVertex(new FramePoint2D(soleFrame, footLength / 2.0, footWidth / 2.0));
      gridBoundaries.update();

      registry = new YoRegistry(namePrefix + name);

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

      if (VISUALIZE)
      {
         cellViz = new YoFramePoint3D[nLengthSubdivisions][nWidthSubdivisions];
         for (int i = 0; i < cellViz.length; i++)
         {
            for (int j = 0; j < cellViz[0].length; j++)
            {
               String namePrefix2 = "CellViz_X" + String.valueOf(i) + "Y" + String.valueOf(j);
               YoFramePoint3D pointForViz = new YoFramePoint3D(namePrefix + namePrefix2, ReferenceFrame.getWorldFrame(), registry);
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
      YoVariableChangedListener changedGridSizeListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            counterGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
            occupancyGrid.reshape(nLengthSubdivisions.getIntegerValue(), nWidthSubdivisions.getIntegerValue());
            cellSize.setX(footLength / nLengthSubdivisions.getIntegerValue());
            cellSize.setY(footWidth / nWidthSubdivisions.getIntegerValue());
            cellArea.set(cellSize.getX() * cellSize.getY());
         }
      };
      nLengthSubdivisions.addListener(changedGridSizeListener);
      nWidthSubdivisions.addListener(changedGridSizeListener);
      changedGridSizeListener.changed(null);

      YoVariableChangedListener changedThresholdForCellActivationListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
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
      thresholdForCellActivation.addListener(changedThresholdForCellActivationListener);
      changedThresholdForCellActivationListener.changed(null);

      YoVariableChangedListener resetGridListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (resetGridToEmpty.getBooleanValue())
            {
               reset();
            }
            resetGridToEmpty.set(false);
         }
      };
      resetGridToEmpty.addListener(resetGridListener);
      resetGridListener.changed(null);
   }

   private final FramePoint3D cellPosition = new FramePoint3D();

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
            if (VISUALIZE)
            {
               getCellCenter(cellCenter, xIndex, yIndex);
               cellPosition.setIncludingFrame(cellCenter, 0.0);
               cellViz[xIndex][yIndex].setMatchingFrame(cellPosition);
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

   public boolean isCellAtLocationOccupied(FramePoint2D location)
   {
      location.checkReferenceFrameMatch(soleFrame);
      tempPoint.sub(location, gridOrigin);

      int xIndex = findXIndex(tempPoint.getX());
      int yIndex = findYIndex(tempPoint.getY());
      if (checkIfIndicesAreValid(xIndex, yIndex))
         return occupancyGrid.get(xIndex, yIndex) > 0.9;
      else
         return false;
   }

   public void getCellCenter(FramePoint2D cellCenter, int xIndex, int yIndex)
   {
      double x = getXCoordinateInSoleFrame(xIndex);
      double y = getYCoordinateInSoleFrame(yIndex);

      cellCenter.setIncludingFrame(soleFrame, x, y);
   }

   private double getXCoordinateInSoleFrame(int xIndex)
   {
      return (xIndex + 0.5) * cellSize.getX() + gridOrigin.getX();
   }

   private double getYCoordinateInSoleFrame(int yIndex)
   {
      return (yIndex + 0.5) * cellSize.getY() + gridOrigin.getY();
   }

   public boolean findCenterOfClosestCell(FramePoint2D centerOfClosestCellToPack, FramePoint2D closestToPoint)
   {
      int xIndex = findXIndex(closestToPoint.getX());
      int yIndex = findYIndex(closestToPoint.getY());

      if (!checkIfIndicesAreValid(xIndex, yIndex))
         return false;

      getCellCenter(centerOfClosestCellToPack, xIndex, yIndex);
      return true;
   }

   private final FrameLine2D shiftedLine = new FrameLine2D();
   private final FrameVector2D shiftedLineVector = new FrameVector2D();
   private final FramePoint2D shiftedLinePoint = new FramePoint2D();
   private final FrameVector2D shiftingVector = new FrameVector2D();
   private final FramePoint2D cellCenter = new FramePoint2D();

   /**
    * This algorithm is stupid because it checks for every cell if the cell is on the right side and if
    * it is activated.
    * 
    * @param frameLine
    * @param sideToLookAt
    * @return
    */
   public int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2D frameLine, RobotSide sideToLookAt)
   {
      return computeNumberOfCellsOccupiedOnSideOfLine(frameLine, sideToLookAt, 0.0);
   }

   public int computeNumberOfCellsOccupiedOnSideOfLine(FrameLine2DReadOnly frameLine, RobotSide sideToLookAt, double minDistanceFromLine)
   {
      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.
      frameLine.checkReferenceFrameMatch(soleFrame);
      shiftingVector.setIncludingFrame(frameLine.getDirection());

      shiftedLinePoint.setIncludingFrame(frameLine.getPoint());
      shiftedLineVector.setIncludingFrame(frameLine.getDirection());

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      EuclidGeometryTools.perpendicularVector2D(shiftingVector, shiftingVector);
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
            if (shiftedLine.isPointOnSideOfLine(cellCenter, sideToLookAt == RobotSide.LEFT))
               numberOfCellsActivatedOnSideToLookAt += occupancyGrid.get(xIndex, yIndex);
         }
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }

   /**
    * This algorithm is smarter since it doesn't go through all the cells, but it probably doesn't work
    * :/
    * 
    * @param frameLine
    * @param sideToLookAt
    * @return
    */
   public int computeNumberOfCellsOccupiedOnSideOfLineSmarter(FrameLine2D frameLine, RobotSide sideToLookAt)
   {
      int returnFailure = -1;

      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.

      frameLine.checkReferenceFrameMatch(soleFrame);
      shiftingVector.setIncludingFrame(frameLine.getDirection());

      shiftedLinePoint.setIncludingFrame(frameLine.getPoint());
      shiftedLineVector.setIncludingFrame(frameLine.getDirection());

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      EuclidGeometryTools.perpendicularVector2D(shiftingVector, shiftingVector);
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
      FramePoint2DBasics[] intersections = gridBoundaries.intersectionWith(shiftedLine);

      if (intersections == null || intersections.length == 1)
         return returnFailure;

      FrameVector2D intersectionsVector = new FrameVector2D(soleFrame);
      intersectionsVector.sub(intersections[1], intersections[0]);

      FramePoint2D temp = new FramePoint2D(soleFrame);
      FramePoint2D cellCenter = new FramePoint2D();
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

            if (VISUALIZE)
            {
               if (isCellOccupied(xIndex, yIndex))
               {
                  getCellCenter(cellCenter, xIndex, yIndex);
                  cellPosition.setIncludingFrame(cellCenter, 0.0);
                  cellViz[xIndex][yIndex].setMatchingFrame(cellPosition);
               }
               else
               {
                  cellViz[xIndex][yIndex].setToNaN();
               }
            }
         }
      }
   }

   private final FramePoint2D tempCellCenter = new FramePoint2D();

   public void computeConvexHull(FrameConvexPolygon2D convexHullToPack)
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
   private final DMatrixRMaj pointCloud = new DMatrixRMaj(0, 0);
   private final Point3D tempPoint3d = new Point3D();
   private final FramePoint2D lineOrigin = new FramePoint2D();
   private final Vector3D tempVector3d = new Vector3D();
   private final FrameVector2D lineDirection = new FrameVector2D();

   private final FramePoint2D pointA = new FramePoint2D();
   private final FramePoint2D pointB = new FramePoint2D();

   public boolean fitLineToData(FrameLine2D lineToPack)
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
         lineToPack.setIncludingFrame(pointA, pointB);
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
      lineToPack.getPoint().set(lineOrigin);
      lineToPack.getDirection().set(lineDirection);
      return true;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (!VISUALIZE)
         return null;
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < cellViz.length; i++)
      {
         for (int j = 0; j < cellViz[0].length; j++)
         {
            String namePrefix = "CellViz_X" + String.valueOf(i) + "Y" + String.valueOf(j);
            YoGraphicPoint3DDefinition yoGraphicPoint3D = YoGraphicDefinitionFactory.newYoGraphicPoint3D(namePrefix,
                                                                                                         cellViz[i][j],
                                                                                                         0.008,
                                                                                                         ColorDefinitions.Orange());
            group.addChild(yoGraphicPoint3D);
            group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D(yoGraphicPoint3D, DefaultPoint2DGraphic.CIRCLE));
         }
      }
      return group;
   }
}
