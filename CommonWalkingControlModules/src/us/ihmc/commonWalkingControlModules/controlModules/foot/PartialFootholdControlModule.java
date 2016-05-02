package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
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
   private final String name = getClass().getSimpleName();

   public enum PartialFootholdState
   {
      FULL, PARTIAL
   };

   private final YoVariableRegistry registry;

   private final EnumYoVariable<PartialFootholdState> footholdState;

   private final FootRotationCalculator footRotationCalculator;
   private final FootCoPOccupancyGrid footCoPOccupancyGrid;

   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2d defaultFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygon;
   private final FrameConvexPolygon2d backupFootPolygon;
   private final FrameConvexPolygon2d unsafePolygon;
   private final YoFrameConvexPolygon2d yoUnsafePolygon;

   private final IntegerYoVariable shrinkMaxLimit;
   private final IntegerYoVariable shrinkCounter;

   private final IntegerYoVariable numberOfVerticesRemoved;
   private final IntegerYoVariable numberOfCellsOccupiedOnSideOfLine;

   private final IntegerYoVariable thresholdForCoPCellOccupancy;
   private final IntegerYoVariable thresholdForCoPRegionOccupancy;
   private final DoubleYoVariable distanceFromLineOfRotationToComputeCoPOccupancy;

   private final BooleanYoVariable doPartialFootholdDetection;

   private final IntegerYoVariable confusingCutIndex;

   private final FrameLine2d lineOfRotation;

   private final BooleanYoVariable useCoPOccupancyGrid;

   private final int footCornerPoints;
   private Point2d newVertex = new Point2d();

   public PartialFootholdControlModule(String namePrefix, double dt, ContactablePlaneBody contactableFoot, TwistCalculator twistCalculator,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      footCornerPoints = contactableFoot.getTotalNumberOfContactPoints();
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
      shrunkFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      backupFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      lineOfRotation = new FrameLine2d(soleFrame);

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      footholdState = new EnumYoVariable<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2d(namePrefix + "UnsafeFootPolygon", "", ReferenceFrame.getWorldFrame(), 10, registry);

      shrinkMaxLimit = new IntegerYoVariable(namePrefix + "MaximumNumberOfFootShrink", registry);
      shrinkMaxLimit.set(6);
      shrinkCounter = new IntegerYoVariable(namePrefix + "FootShrinkCounter", registry);

      confusingCutIndex = new IntegerYoVariable(namePrefix + "ConfusingCutIndex", registry);

      numberOfVerticesRemoved = new IntegerYoVariable(namePrefix + "NumberOfVerticesRemoved", registry);
      numberOfCellsOccupiedOnSideOfLine = new IntegerYoVariable(namePrefix + "NumberOfCellsOccupiedOnSideOfLine", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon yoGraphicPolygon = new YoArtifactPolygon(namePrefix + "UnsafeRegion", yoUnsafePolygon, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact("Partial Foothold", yoGraphicPolygon);
      }

      footRotationCalculator = new FootRotationCalculator(namePrefix, dt, contactableFoot, twistCalculator, yoGraphicsListRegistry, registry);
      footCoPOccupancyGrid = new FootCoPOccupancyGrid(namePrefix, soleFrame, walkingControllerParameters.getFootLength(),
            walkingControllerParameters.getFootWidth(), 20, 10, yoGraphicsListRegistry, registry);

      thresholdForCoPCellOccupancy = new IntegerYoVariable(namePrefix + "ThresholdForCoPCellOccupancy", registry);
      thresholdForCoPCellOccupancy.set(3);
      footCoPOccupancyGrid.setThresholdForCellActivation(thresholdForCoPCellOccupancy.getIntegerValue());

      thresholdForCoPRegionOccupancy = new IntegerYoVariable(namePrefix + "ThresholdForCoPRegionOccupancy", registry);
      thresholdForCoPRegionOccupancy.set(2);
      distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleYoVariable(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry);
      distanceFromLineOfRotationToComputeCoPOccupancy.set(0.02);

      doPartialFootholdDetection = new BooleanYoVariable(namePrefix + "DoPartialFootholdDetection", registry);
      doPartialFootholdDetection.set(false);

      useCoPOccupancyGrid = new BooleanYoVariable(namePrefix + "UseCoPOccupancyGrid", registry);
      useCoPOccupancyGrid.set(true);
   }

   public void compute(FramePoint2d desiredCenterOfPressure, FramePoint2d centerOfPressure)
   {
      if (desiredCenterOfPressure.containsNaN() || centerOfPressure.containsNaN())
      {
         doNothing();
         return;
      }

      unsafePolygon.setIncludingFrameAndUpdate(shrunkFootPolygon);
      footCoPOccupancyGrid.registerCenterOfPressureLocation(centerOfPressure);

      footRotationCalculator.compute(desiredCenterOfPressure, centerOfPressure);

      if (footRotationCalculator.isFootRotating())
      {
         footRotationCalculator.getLineOfRotation(lineOfRotation);

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
   }

   private void computeShrunkFoothold(FramePoint2d desiredCenterOfPressure)
   {
      boolean wasCoPInThatRegion = false;
      if (useCoPOccupancyGrid.getBooleanValue()) {
         numberOfCellsOccupiedOnSideOfLine.set(footCoPOccupancyGrid.computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation, RobotSide.RIGHT,
               distanceFromLineOfRotationToComputeCoPOccupancy.getDoubleValue()));
         wasCoPInThatRegion = numberOfCellsOccupiedOnSideOfLine.getIntegerValue() >= thresholdForCoPRegionOccupancy.getIntegerValue();
      }

      if (unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3) && !wasCoPInThatRegion)
      {
         backupFootPolygon.set(shrunkFootPolygon);
         ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkFootPolygon, RobotSide.RIGHT);
         unsafePolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         yoUnsafePolygon.setFrameConvexPolygon2d(unsafePolygon);
      }
      else
      {
         doNothing();
      }
   }

   private final FramePoint tempPosition = new FramePoint();

   public boolean applyShrunkPolygon(YoPlaneContactState contactStateToModify)
   {
      if (!doPartialFootholdDetection.getBooleanValue())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      if (footholdState.getEnumValue() == PartialFootholdState.FULL)
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      if (shrinkCounter.getIntegerValue() >= shrinkMaxLimit.getIntegerValue())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      int newFootCornerPoints = shrunkFootPolygon.getNumberOfVertices();
      if (newFootCornerPoints == footCornerPoints)
      {
         // everything is well
      }
      else if (newFootCornerPoints == footCornerPoints + 1)
      {
         // we cut off an edge
         // remove one corner by merging the two closest vertices
         int removeVertex = -1;
         double shortestEdgeLength = Double.POSITIVE_INFINITY;
         Point2d lastVertex = shrunkFootPolygon.getVertex(0);
         for (int i = 1; i < newFootCornerPoints+1; i++)
         {
            Point2d nextVertex = null;
            if (i == newFootCornerPoints)
            {
               nextVertex = shrunkFootPolygon.getVertex(0);
            }
            else
            {
               nextVertex = shrunkFootPolygon.getVertex(i);
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

         Point2d vertexA = shrunkFootPolygon.getVertex(idx1);
         Point2d vertexB = shrunkFootPolygon.getVertex(idx2);
         newVertex.interpolate(vertexA, vertexB, 0.5);

         shrunkFootPolygon.removeVertex(idx1);
         shrunkFootPolygon.removeVertex(idx2);
         shrunkFootPolygon.addVertex(newVertex);
         shrunkFootPolygon.update();
      }
      else if (newFootCornerPoints < footCornerPoints)
      {
         // we cut off too many corners
         // add vertices in the longest edges of the polygon
         int pointsToAdd = footCornerPoints - newFootCornerPoints;
         while (pointsToAdd > 0) {
            int index = -1;
            double longestEdgeLength = Double.NEGATIVE_INFINITY;
            Point2d lastVertex = shrunkFootPolygon.getVertex(0);
            for (int i = 1; i < newFootCornerPoints+1; i++)
            {
               Point2d nextVertex = null;
               if (i == newFootCornerPoints)
               {
                  nextVertex = shrunkFootPolygon.getVertex(0);
               }
               else
               {
                  nextVertex = shrunkFootPolygon.getVertex(i);
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

            Point2d vertexA = shrunkFootPolygon.getVertex(idx1);
            Point2d vertexB = shrunkFootPolygon.getVertex(idx2);
            newVertex.interpolate(vertexA, vertexB, 0.5);

            shrunkFootPolygon.addVertex(newVertex);
            shrunkFootPolygon.update();

            pointsToAdd--;
         }
      }
      else
      {
         // cutting a convex polygon with a line should never result in the number of vertices increasing by more then one.
         throw new RuntimeException("This is not possible.");
      }

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();

      if (contactPoints.size() != shrunkFootPolygon.getNumberOfVertices())
      {
         shrunkFootPolygon.set(backupFootPolygon);
         return false;
      }

      for (int i = 0; i < shrunkFootPolygon.getNumberOfVertices(); i++)
      {
         shrunkFootPolygon.getFrameVertexXY(i, tempPosition);
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
      footRotationCalculator.reset();
      footCoPOccupancyGrid.reset();
      footCoPOccupancyGrid.setThresholdForCellActivation(thresholdForCoPCellOccupancy.getIntegerValue());
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
