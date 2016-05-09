package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.EnumMap;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

public class PartialFootholdControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();

   public enum PartialFootholdState
   {
      FULL, PARTIAL
   };

   private final YoVariableRegistry registry;

   private final EnumYoVariable<PartialFootholdState> footholdState;

   public enum RotationCalculatorType
   {
      VELOCITY, GEOMETRY;
      public static RotationCalculatorType[] values = values();
   }
   private final EnumMap<RotationCalculatorType, FootRotationCalculator> rotationCalculators = new EnumMap<>(RotationCalculatorType.class);
   private final EnumYoVariable<RotationCalculatorType> rotationCalculatorType;

   private final RotationVerificator rotationVerificator;

   private final FootCoPOccupancyGrid footCoPOccupancyGrid;

//   private final HighLevelHumanoidControllerToolbox momentumBasedController;
//   private final RobotSide robotSide;
   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2d defaultFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygonInWorld;
   private final YoFrameConvexPolygon2d yoShrunkFootPolygon;
   private final FrameConvexPolygon2d controllerFootPolygon;
   private final FrameConvexPolygon2d backupFootPolygon;
   private final FrameConvexPolygon2d unsafePolygon;
   private final YoFrameConvexPolygon2d yoUnsafePolygon;

   private final IntegerYoVariable shrinkMaxLimit;
   private final IntegerYoVariable shrinkCounter;

   private final IntegerYoVariable numberOfVerticesRemoved;
   private final IntegerYoVariable numberOfCellsOccupiedOnSideOfLine;

   private final IntegerYoVariable thresholdForCoPRegionOccupancy;
   private final DoubleYoVariable distanceFromLineOfRotationToComputeCoPOccupancy;

   private final BooleanYoVariable doPartialFootholdDetection;

   private final IntegerYoVariable confusingCutIndex;

   private final FrameLine2d lineOfRotation;

   private final BooleanYoVariable useCoPOccupancyGrid;

   private final int footCornerPoints;
   private Point2d newVertex = new Point2d();

   /**
    * Variables for checking the area of the unsafe part of the foothold.
    */
   private final DoubleYoVariable unsafeArea;
   private final DoubleYoVariable minAreaToConsider;
   private final BooleanYoVariable unsafeAreaAboveThreshold;

   public PartialFootholdControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ContactableFoot contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      String namePrefix = contactableFoot.getRigidBody().getName();
//      this.momentumBasedController = momentumBasedController;
//      this.robotSide = robotSide;

      footCornerPoints = contactableFoot.getTotalNumberOfContactPoints();
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
      shrunkFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      shrunkFootPolygonInWorld = new FrameConvexPolygon2d(defaultFootPolygon);
      controllerFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      backupFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      lineOfRotation = new FrameLine2d(soleFrame);

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);
      ExplorationParameters explorationParameters = walkingControllerParameters.getOrCreateExplorationParameters(registry);

      footholdState = new EnumYoVariable<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2d(namePrefix + "UnsafeFootPolygon", "", worldFrame, 10, registry);
      yoShrunkFootPolygon = new YoFrameConvexPolygon2d(namePrefix + "ShrunkFootPolygon", "", worldFrame, 20, registry);

      shrinkCounter = new IntegerYoVariable(namePrefix + "FootShrinkCounter", registry);

      confusingCutIndex = new IntegerYoVariable(namePrefix + "ConfusingCutIndex", registry);

      numberOfVerticesRemoved = new IntegerYoVariable(namePrefix + "NumberOfVerticesRemoved", registry);
      numberOfCellsOccupiedOnSideOfLine = new IntegerYoVariable(namePrefix + "NumberOfCellsOccupiedOnSideOfLine", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon yoGraphicPolygon = new YoArtifactPolygon(namePrefix + "UnsafeRegion", yoUnsafePolygon, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact("Partial Foothold", yoGraphicPolygon);

         YoArtifactPolygon yoShrunkPolygon = new YoArtifactPolygon(namePrefix + "ShrunkPolygon", yoShrunkFootPolygon, Color.CYAN, false);
         yoGraphicsListRegistry.registerArtifact("Shrunk Polygon", yoShrunkPolygon);
      }

      footCoPOccupancyGrid = new FootCoPOccupancyGrid(namePrefix, soleFrame, 20, 10, walkingControllerParameters, yoGraphicsListRegistry, registry);

      shrinkMaxLimit = explorationParameters.getShrinkMaxLimit();
      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();
      useCoPOccupancyGrid = explorationParameters.getUseCopOccupancyGrid();
      rotationCalculatorType = explorationParameters.getRotationCalculatorType();
      minAreaToConsider = explorationParameters.getMinAreaToConsider();

      doPartialFootholdDetection = new BooleanYoVariable(namePrefix + "DoPartialFootholdDetection", registry);
      doPartialFootholdDetection.set(false);

      double dt = momentumBasedController.getControlDT();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();

      FootRotationCalculator velocityFootRotationCalculator =
            new VelocityFootRotationCalculator(namePrefix, dt, contactableFoot, twistCalculator, explorationParameters, yoGraphicsListRegistry, registry);
      FootRotationCalculator geometricFootRotationCalculator =
            new GeometricFootRotationCalculator(namePrefix, contactableFoot, explorationParameters, yoGraphicsListRegistry, registry);
      rotationCalculators.put(RotationCalculatorType.VELOCITY, velocityFootRotationCalculator);
      rotationCalculators.put(RotationCalculatorType.GEOMETRY, geometricFootRotationCalculator);

      rotationVerificator = new RotationVerificator(namePrefix, contactableFoot, explorationParameters, registry);

      unsafeArea = new DoubleYoVariable(namePrefix + "UnsafeArea", registry);
      unsafeAreaAboveThreshold = new BooleanYoVariable(namePrefix + "UnsafeAreaAboveThreshold", registry);
   }

   public void compute(FramePoint2d desiredCenterOfPressure, FramePoint2d centerOfPressure)
   {
      footCoPOccupancyGrid.update();

      if (desiredCenterOfPressure.containsNaN() || centerOfPressure.containsNaN())
      {
         doNothing();
         return;
      }

      unsafePolygon.setIncludingFrameAndUpdate(shrunkFootPolygon);
      footCoPOccupancyGrid.registerCenterOfPressureLocation(centerOfPressure);

      for (RotationCalculatorType calculatorType : RotationCalculatorType.values)
      {
         if (!rotationCalculators.containsKey(calculatorType)) continue;
         rotationCalculators.get(calculatorType).compute(desiredCenterOfPressure, centerOfPressure);
      }
      FootRotationCalculator activeCalculator = rotationCalculators.get(rotationCalculatorType.getEnumValue());

      activeCalculator.getLineOfRotation(lineOfRotation);
      boolean verified = rotationVerificator.isRotating(centerOfPressure, desiredCenterOfPressure, lineOfRotation);

      if (activeCalculator.isFootRotating() && verified)
      {
         numberOfVerticesRemoved.set(ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, unsafePolygon, RobotSide.LEFT));

         if (numberOfVerticesRemoved.getIntegerValue() <= 0)
         {
            confusingCutIndex.increment();

//            System.out.println("\nCutting polygon but no vertices removed?");
//            System.out.println("confusingCutIndex = " + confusingCutIndex.getIntegerValue());
//            System.out.println("lineOfRotation = " + lineOfRotation);
//            System.out.println("unsafePolygon = " + unsafePolygon);
//            System.out.println("shrunkFootPolygon = " + shrunkFootPolygon);

            doNothing();
         }
         else
         {
            footholdState.set(PartialFootholdState.PARTIAL);
            computeShrunkFoothold(desiredCenterOfPressure);
         }
      }
      else
      {
         doNothing();
      }
   }

   public void getShrunkPolygonCentroid(FramePoint2d centroidToPack)
   {
      shrunkFootPolygon.getCentroid(centroidToPack);
   }

   private void doNothing()
   {
      footholdState.set(PartialFootholdState.FULL);
      yoUnsafePolygon.hide();
      shrunkFootPolygonInWorld.setIncludingFrame(shrunkFootPolygon);
      shrunkFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
      yoShrunkFootPolygon.setFrameConvexPolygon2d(shrunkFootPolygonInWorld);
      unsafeArea.set(0.0);
   }

   private void computeShrunkFoothold(FramePoint2d desiredCenterOfPressure)
   {
      boolean wasCoPInThatRegion = false;
      if (useCoPOccupancyGrid.getBooleanValue()) {
         numberOfCellsOccupiedOnSideOfLine.set(footCoPOccupancyGrid.computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation, RobotSide.RIGHT,
               distanceFromLineOfRotationToComputeCoPOccupancy.getDoubleValue()));
         wasCoPInThatRegion = numberOfCellsOccupiedOnSideOfLine.getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();
      }

      unsafeArea.set(unsafePolygon.getArea());
      boolean areaBigEnough = unsafeArea.getDoubleValue() >= minAreaToConsider.getDoubleValue();
      unsafeAreaAboveThreshold.set(areaBigEnough);

      if (unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3) && !wasCoPInThatRegion && areaBigEnough)
      {
         backupFootPolygon.set(shrunkFootPolygon);
         ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkFootPolygon, RobotSide.RIGHT);
         unsafePolygon.changeFrameAndProjectToXYPlane(worldFrame);
         yoUnsafePolygon.setFrameConvexPolygon2d(unsafePolygon);

         shrunkFootPolygonInWorld.setIncludingFrame(shrunkFootPolygon);
         shrunkFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
         yoShrunkFootPolygon.setFrameConvexPolygon2d(shrunkFootPolygonInWorld);
      }
      else
      {
         doNothing();
      }
   }

   private final FramePoint tempPosition = new FramePoint();

   public boolean applyShrunkPolygon(YoPlaneContactState contactStateToModify)
   {
      // if we are not doing partial foothold detection exit
      if (!doPartialFootholdDetection.getBooleanValue())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      // if the module did not find a partial foothold exit
      if (footholdState.getEnumValue() == PartialFootholdState.FULL)
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      // if we shrunk the foothold too many times exit
      if (shrinkCounter.getIntegerValue() >= shrinkMaxLimit.getIntegerValue())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      controllerFootPolygon.setIncludingFrame(shrunkFootPolygon);
      int newFootCornerPoints = controllerFootPolygon.getNumberOfVertices();
      if (newFootCornerPoints == footCornerPoints)
      {
         // everything is well
      }
      else if (newFootCornerPoints == footCornerPoints + 1)
      {
         // we cut off a corner
         // remove one corner by merging the two closest vertices
         int removeVertex = -1;
         double shortestEdgeLength = Double.POSITIVE_INFINITY;
         Point2d lastVertex = controllerFootPolygon.getVertex(0);
         for (int i = 1; i < newFootCornerPoints+1; i++)
         {
            Point2d nextVertex = null;
            if (i == newFootCornerPoints)
            {
               nextVertex = controllerFootPolygon.getVertex(0);
            }
            else
            {
               nextVertex = controllerFootPolygon.getVertex(i);
            }
            double edgeLength = lastVertex.distance(nextVertex);
            if (edgeLength < shortestEdgeLength)
            {
               shortestEdgeLength = edgeLength;
               removeVertex = i;
            }
            lastVertex = nextVertex;
         }

         if (removeVertex < 1)
         {
            throw new RuntimeException("Did not find an edge in support polygon.");
         }

         int idx1 = -1;
         int idx2 = -1;
         if (removeVertex == newFootCornerPoints)
         {
            idx1 = newFootCornerPoints-1;
            idx2 = 0;
         }
         else
         {
            idx1 = removeVertex;
            idx2 = removeVertex-1;
         }

         Point2d vertexA = controllerFootPolygon.getVertex(idx1);
         Point2d vertexB = controllerFootPolygon.getVertex(idx2);
         newVertex.interpolate(vertexA, vertexB, 0.5);

         controllerFootPolygon.removeVertex(idx1);
         controllerFootPolygon.removeVertex(idx2);
         controllerFootPolygon.addVertex(newVertex);
         controllerFootPolygon.update();
      }
      else if (newFootCornerPoints < footCornerPoints)
      {
         // we cut off too many corners
         // add vertices in the longest edges of the polygon
         int pointsToAdd = footCornerPoints - newFootCornerPoints;
         while (pointsToAdd > 0) {
            int index = -1;
            double longestEdgeLength = Double.NEGATIVE_INFINITY;
            Point2d lastVertex = controllerFootPolygon.getVertex(0);
            for (int i = 1; i < newFootCornerPoints+1; i++)
            {
               Point2d nextVertex = null;
               if (i == newFootCornerPoints)
               {
                  nextVertex = controllerFootPolygon.getVertex(0);
               }
               else
               {
                  nextVertex = controllerFootPolygon.getVertex(i);
               }

               double edgeLength = lastVertex.distance(nextVertex);
               if (edgeLength > longestEdgeLength)
               {
                  longestEdgeLength = edgeLength;
                  index = i;
               }
               lastVertex = nextVertex;
            }

            if (index < 1)
            {
               throw new RuntimeException("Did not find an edge in support polygon.");
            }

            int idx1 = -1;
            int idx2 = -1;
            if (index == newFootCornerPoints)
            {
               idx1 = newFootCornerPoints-1;
               idx2 = 0;
            }
            else
            {
               idx1 = index;
               idx2 = index-1;
            }

            Point2d vertexA = controllerFootPolygon.getVertex(idx1);
            Point2d vertexB = controllerFootPolygon.getVertex(idx2);
            newVertex.interpolate(vertexA, vertexB, 0.5);

            controllerFootPolygon.addVertex(newVertex);
            controllerFootPolygon.update();

            pointsToAdd--;
         }
      }
      else
      {
         // cutting a convex polygon with a line should never result in the number of vertices increasing by more then one.
         throw new RuntimeException("This is not possible.");
      }

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();
      if (contactPoints.size() != controllerFootPolygon.getNumberOfVertices())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      for (int i = 0; i < controllerFootPolygon.getNumberOfVertices(); i++)
      {
         controllerFootPolygon.getFrameVertexXY(i, tempPosition);
         contactPoints.get(i).setPosition(tempPosition);
      }

      backupFootPolygon.set(shrunkFootPolygon);
      shrinkCounter.increment();
      return true;
   }

   public void reset()
   {
      shrinkCounter.set(0);
      footholdState.set(null);
      yoUnsafePolygon.hide();
      yoShrunkFootPolygon.hide();
      for (RotationCalculatorType calculatorType : RotationCalculatorType.values)
      {
         if (!rotationCalculators.containsKey(calculatorType)) continue;
         rotationCalculators.get(calculatorType).reset();
      }
      footCoPOccupancyGrid.reset();
      shrunkFootPolygon.setIncludingFrameAndUpdate(defaultFootPolygon);
      backupFootPolygon.setIncludingFrameAndUpdate(defaultFootPolygon);
   }

   public void projectOntoShrunkenPolygon(FramePoint2d pointToProject)
   {
      shrunkFootPolygon.orthogonalProjection(pointToProject);
   }

   public void getSupportPolygon(FrameConvexPolygon2d polygonToPack)
   {
      polygonToPack.setIncludingFrame(shrunkFootPolygon);
   }
}
