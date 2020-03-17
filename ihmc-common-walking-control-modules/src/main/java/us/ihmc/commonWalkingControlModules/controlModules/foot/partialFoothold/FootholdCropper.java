package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.List;

public class FootholdCropper
{
   private static final double defaultThresholdForMeasuredCellActivation = 1.0;
   private static final double defaultMeasuredDecayRatePerSecond = 0.0;// 0.2;

   private final FrameConvexPolygon2D defaultFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygonInWorld;
   private final FrameConvexPolygon2D controllerFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D controllerFootPolygonInWorld = new FrameConvexPolygon2D();

   private final DoubleProvider distanceFromRotationToCrop;
   private final DoubleProvider minAreaToConsider;
   private final YoBoolean hasEnoughAreaToCrop;

   private final OccupancyGrid measuredCoPOccupancy;
   private final OccupancyGrid desiredCoPOccupancy;
   private final FootCoPOccupancyCalculator footCoPOccupancyGrid;
   private final FootCoPHullCalculator footCoPHullCropper;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean doPartialFootholdDetection;
   private final YoBoolean applyPartialFootholds;
   private final IntegerProvider shrinkMaxLimit;
   private final YoInteger shrinkCounter;

   private final YoBoolean shouldShrinkFoothold;

   private final YoEnum<RobotSide> sideOfFootToCrop;
   private final int numberOfFootCornerPoints;

   private final CropVerifier verifier;
   private final OccupancyGridVisualizer measuredVisualizer;
   private final OccupancyGridVisualizer desiredVisualizer;

   public FootholdCropper(String namePrefix,
                          ContactableFoot contactableFoot,
                          FootholdRotationParameters rotationParameters,
                          double dt,
                          YoVariableRegistry parentRegistry,
                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
      numberOfFootCornerPoints = contactableFoot.getTotalNumberOfContactPoints();

      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());


      shrunkenFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygon", "", soleFrame, 20, registry);
      shrunkenFootPolygonInWorld = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygonInWorld", "", ReferenceFrame.getWorldFrame(), 20, registry);
      shrunkenFootPolygon.set(defaultFootPolygon);

      shouldShrinkFoothold = new YoBoolean(namePrefix + "ShouldShrinkFoothold", registry);

      measuredCoPOccupancy = new OccupancyGrid(namePrefix + "MeasuredCoPOccupancy", soleFrame, registry);
      desiredCoPOccupancy = new OccupancyGrid(namePrefix + "DesiredCoPOccupancy", soleFrame, registry);

      double resolution = 0.005;
      measuredCoPOccupancy.setCellXSize(resolution);
      measuredCoPOccupancy.setCellYSize(resolution);
      measuredCoPOccupancy.setThresholdForCellOccupancy(defaultThresholdForMeasuredCellActivation);
      measuredCoPOccupancy.setOccupancyDecayRate(1.0 - Math.pow(defaultMeasuredDecayRatePerSecond, dt));
      desiredCoPOccupancy.setCellXSize(resolution);
      desiredCoPOccupancy.setCellYSize(resolution);

      footCoPOccupancyGrid = new FootCoPOccupancyCalculator(namePrefix, measuredCoPOccupancy, rotationParameters, registry);
      footCoPHullCropper = new FootCoPHullCalculator(namePrefix, measuredCoPOccupancy, rotationParameters, registry);

      verifier = new CropVerifier(namePrefix, desiredCoPOccupancy, rotationParameters, registry);

      sideOfFootToCrop = new YoEnum<>(namePrefix + "SideOfFootToCrop", registry, RobotSide.class, true);

      hasEnoughAreaToCrop = new YoBoolean(namePrefix + "HasEnoughAreaToCrop", registry);

      minAreaToConsider = rotationParameters.getMinimumAreaForCropping();
      distanceFromRotationToCrop = rotationParameters.getDistanceFromRotationToCrop();

      doPartialFootholdDetection = new YoBoolean(namePrefix + "DoPartialFootholdDetection", registry);
      applyPartialFootholds = new YoBoolean(namePrefix + "ApplyPartialFootholds", registry);
      doPartialFootholdDetection.set(true);
      applyPartialFootholds.set(false);
      shrinkCounter = new YoInteger(namePrefix + "ShrinkCounter", registry);
      shrinkMaxLimit = rotationParameters.getShrinkMaxLimit();

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();

         YoArtifactPolygon yoShrunkPolygon = new YoArtifactPolygon(namePrefix + "ShrunkPolygon", shrunkenFootPolygonInWorld, Color.BLUE, false);
         yoShrunkPolygon.setVisible(true);
         yoGraphicsListRegistry.registerArtifact(listName, yoShrunkPolygon);

         measuredVisualizer = new OccupancyGridVisualizer(namePrefix + "MeasuaredCoP", measuredCoPOccupancy, 50, YoAppearance.Red(), registry, yoGraphicsListRegistry);
         desiredVisualizer =  new OccupancyGridVisualizer(namePrefix + "DesiredCoP", desiredCoPOccupancy, 50, YoAppearance.Blue(), registry, yoGraphicsListRegistry);
      }
      else
      {
         measuredVisualizer = null;
         desiredVisualizer = null;
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      reset(defaultFootPolygon);
   }

   public void reset(FrameConvexPolygon2DReadOnly polygon)
   {
      sideOfFootToCrop.set(null);
      shrunkenFootPolygon.set(polygon);
      shrunkenFootPolygonInWorld.clear();
      shrinkCounter.set(0);

      measuredCoPOccupancy.reset();
      desiredCoPOccupancy.reset();
      footCoPHullCropper.reset();
      footCoPOccupancyGrid.reset();
      if (measuredVisualizer != null)
         measuredVisualizer.update();
      if (desiredVisualizer != null)
         desiredVisualizer.update();
   }

   public void update(FramePoint2DReadOnly measuredCoP, FramePoint2DReadOnly desiredCoP)
   {
      shouldShrinkFoothold.set(false);
      measuredCoPOccupancy.update();
      desiredCoPOccupancy.update();

      if (!measuredCoP.containsNaN())
         measuredCoPOccupancy.registerPoint(measuredCoP);
      if (!desiredCoP.containsNaN())
         desiredCoPOccupancy.registerPoint(desiredCoP);

      if (measuredVisualizer != null)
         measuredVisualizer.update();
      if (desiredVisualizer != null)
         desiredVisualizer.update();
   }

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final FrameLine2D cropLine = new FrameLine2D();
   private final Vector2D shiftVector = new Vector2D();

   public void computeShrunkenFoothold(FrameLine2DReadOnly lineOfRotation, FramePoint2DReadOnly desiredCoP)
   {
      RobotSide sideOfFootToCropFromOccupancy = footCoPOccupancyGrid.computeSideOfFootholdToCrop(lineOfRotation);
      RobotSide sideOfFootToCropFromHull = footCoPHullCropper.computeSideOfFootholdToCrop(lineOfRotation);

      boolean sidesAreConsistent = sideOfFootToCropFromOccupancy != null && sideOfFootToCropFromHull != null;
      sidesAreConsistent &= sideOfFootToCropFromOccupancy == sideOfFootToCropFromHull;
      hasEnoughAreaToCrop.set(shrunkenFootPolygon.getArea() > minAreaToConsider.getValue());

      if (sidesAreConsistent && hasEnoughAreaToCrop.getBooleanValue())
      {
         sideOfFootToCrop.set(sideOfFootToCropFromOccupancy);
         shouldShrinkFoothold.set(verifier.verifyFootholdCrop(desiredCoP, sideOfFootToCrop.getEnumValue(), lineOfRotation));

         if (shouldShrinkFoothold.getBooleanValue())
         {
            // FIXME this is a work around for a bug in the cut polygon with line class that doens't work with yo frame convex polygons.
            tempPolygon.setIncludingFrame(shrunkenFootPolygon);
            shiftLine(lineOfRotation, cropLine, sideOfFootToCrop.getEnumValue().getOppositeSide(), distanceFromRotationToCrop.getValue());
            convexPolygonTools.cutPolygonWithLine(cropLine, tempPolygon, sideOfFootToCrop.getEnumValue());
            shrunkenFootPolygon.set(tempPolygon);
         }
      }
      else
      {
         shouldShrinkFoothold.set(false);
         sideOfFootToCrop.set(null);
      }
   }

   private void shiftLine(FrameLine2DReadOnly lineToShift, FrameLine2DBasics shiftedLineToPack, RobotSide sideToShift, double distanceToShift)
   {
      EuclidGeometryTools.perpendicularVector2D(lineToShift.getDirection(), shiftVector);
      if (sideToShift == RobotSide.RIGHT)
         shiftVector.negate();

      shiftVector.scale(distanceToShift);

      shiftedLineToPack.setIncludingFrame(lineToShift);
      shiftedLineToPack.getPoint().add(shiftVector);
   }

   public boolean shouldShrinkFoothold()
   {
      return shouldShrinkFoothold.getBooleanValue();
   }

   public boolean applyShrunkenFoothold(YoPlaneContactState contactStateToModify)
   {
      // if we are not doing partial foothold detection exit
      if (!doPartialFootholdDetection.getBooleanValue())
      {
         shrunkenFootPolygon.set(defaultFootPolygon);
         return false;
      }

      if (!shouldShrinkFoothold.getBooleanValue())
      {
         return false;
      }

      // if we shrunk the foothold too many times exit
      if (shrinkCounter.getIntegerValue() >= shrinkMaxLimit.getValue())
      {
         return false;
      }

      // make sure the foot has the right number of contact points
      controllerFootPolygon.setIncludingFrame(shrunkenFootPolygon);
      ConvexPolygonTools.limitVerticesConservative(controllerFootPolygon, numberOfFootCornerPoints);
      controllerFootPolygonInWorld.setIncludingFrame(controllerFootPolygon);
      controllerFootPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      shrunkenFootPolygonInWorld.set(controllerFootPolygonInWorld);

      if (applyPartialFootholds.getBooleanValue())
      {
         List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();
         int i = 0;
         for (; i < controllerFootPolygon.getNumberOfVertices(); i++)
         {
            YoContactPoint contactPoint = contactPoints.get(i);
            contactPoint.setPosition(controllerFootPolygon.getVertex(i));
            contactPoint.setInContact(true);
         }
         for (; i < contactPoints.size(); i++)
         {
            contactPoints.get(i).setInContact(false);
         }
      }

      shrinkCounter.increment();
      return true;
   }
}
