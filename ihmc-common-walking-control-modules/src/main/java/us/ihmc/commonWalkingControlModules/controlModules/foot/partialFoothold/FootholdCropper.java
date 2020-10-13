package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import java.awt.Color;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.occupancyGrid.OccupancyGrid;
import us.ihmc.robotics.occupancyGrid.OccupancyGridVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootholdCropper
{
   private static final double defaultThresholdForMeasuredCellActivation = 1.0;
   private static final double defaultMeasuredDecayRatePerSecond = 0.0;// 0.2;

   private final FootholdRotationParameters rotationParameters;

   private final FrameConvexPolygon2D defaultFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygonInWorld;
   private final FrameConvexPolygon2D controllerFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D controllerFootPolygonInWorld = new FrameConvexPolygon2D();

   private final DoubleProvider distanceFromRotationToCrop;
   private final DoubleProvider minAreaToConsider;
   private final YoBoolean hasEnoughAreaToCrop;

   private final OccupancyGrid measuredCoPOccupancy;
   private final FootCoPOccupancyCalculator footCoPOccupancyGrid;
   private final FootCoPHullCalculator footCoPHullCropper;
   private final FootDropCropper footDropCropper;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final BooleanProvider doPartialFootholdDetection;
   private final BooleanProvider applyPartialFootholds;
   private final IntegerProvider shrinkMaxLimit;
   private final YoInteger shrinkCounter;

   private final YoEnum<RobotSide> sideOfFootToCrop;
   private final int numberOfFootCornerPoints;

   private final OccupancyGridVisualizer measuredVisualizer;

   public FootholdCropper(String namePrefix,
                          ReferenceFrame soleFrame,
                          List<? extends FramePoint2DReadOnly> defaultContactPoints,
                          FootholdRotationParameters rotationParameters,
                          double dt,
                          YoRegistry parentRegistry,
                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.rotationParameters = rotationParameters;

      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(defaultContactPoints));
      numberOfFootCornerPoints = defaultContactPoints.size();

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      shrunkenFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygon", "", soleFrame, 20, registry);
      shrunkenFootPolygonInWorld = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygonInWorld", "", ReferenceFrame.getWorldFrame(), 20, registry);
      shrunkenFootPolygon.set(defaultFootPolygon);

      measuredCoPOccupancy = new OccupancyGrid(namePrefix + "MeasuredCoPOccupancy", soleFrame, registry);

      double resolution = 0.005;
      measuredCoPOccupancy.setCellSize(resolution);
      measuredCoPOccupancy.setThresholdForCellOccupancy(defaultThresholdForMeasuredCellActivation);
      measuredCoPOccupancy.setOccupancyDecayRate(1.0 - Math.pow(defaultMeasuredDecayRatePerSecond, dt));

      footCoPOccupancyGrid = new FootCoPOccupancyCalculator(namePrefix, measuredCoPOccupancy, rotationParameters, registry);
      footCoPHullCropper = new FootCoPHullCalculator(namePrefix, measuredCoPOccupancy, rotationParameters, registry);
      footDropCropper = new FootDropCropper(namePrefix, soleFrame, rotationParameters, registry);

      sideOfFootToCrop = new YoEnum<>(namePrefix + "SideOfFootToCrop", registry, RobotSide.class, true);

      hasEnoughAreaToCrop = new YoBoolean(namePrefix + "HasEnoughAreaToCrop", registry);

      minAreaToConsider = rotationParameters.getMinimumAreaForCropping();
      distanceFromRotationToCrop = rotationParameters.getDistanceFromRotationToCrop();

      doPartialFootholdDetection = rotationParameters.getDoPartialFootholdDetection();
      applyPartialFootholds = rotationParameters.getApplyPartialFootholds();
      shrinkCounter = new YoInteger(namePrefix + "ShrinkCounter", registry);
      shrinkMaxLimit = rotationParameters.getShrinkMaxLimit();

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();

         YoArtifactPolygon yoShrunkPolygon = new YoArtifactPolygon(namePrefix + "ShrunkPolygon", shrunkenFootPolygonInWorld, Color.BLUE, false);
         yoShrunkPolygon.setVisible(true);
         yoGraphicsListRegistry.registerArtifact(listName, yoShrunkPolygon);

//         measuredVisualizer = new OccupancyGridVisualizer(namePrefix + "MeasuredCoP",
//                                                          measuredCoPOccupancy,
//                                                          50,
//                                                          YoAppearance.Red(),
//                                                          registry,
//                                                          yoGraphicsListRegistry);
         measuredVisualizer = null;
      }
      else
      {
         measuredVisualizer = null;
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
      footCoPHullCropper.reset();
      footCoPOccupancyGrid.reset();
      footDropCropper.reset();
      if (measuredVisualizer != null)
         measuredVisualizer.update();
   }

   public void update(FramePoint2DReadOnly measuredCoP)
   {
      measuredCoPOccupancy.update();

      if (!measuredCoP.containsNaN())
         measuredCoPOccupancy.registerPoint(measuredCoP);

      if (measuredVisualizer != null)
         measuredVisualizer.update();
   }

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final FrameLine2D cropLine = new FrameLine2D();
   private final Vector2D shiftVector = new Vector2D();

   public RobotSide computeSideToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      hasEnoughAreaToCrop.set(shrunkenFootPolygon.getArea() > minAreaToConsider.getValue());

      RobotSide sideOfFootToCrop = getSideToCrop(lineOfRotation);
      if (sideOfFootToCrop != null && hasEnoughAreaToCrop.getBooleanValue())
      {
         this.sideOfFootToCrop.set(sideOfFootToCrop);
      }
      else
      {
         this.sideOfFootToCrop.set(null);
      }

      return this.sideOfFootToCrop.getEnumValue();
   }

   private RobotSide getSideToCrop(FrameLine2DReadOnly lineOfRotation)
   {
      RobotSide sideOfFootToCropFromOccupancy = footCoPOccupancyGrid.computeSideOfFootholdToCrop(lineOfRotation);
      RobotSide sideOfFootToCropFromHull = footCoPHullCropper.computeSideOfFootholdToCrop(lineOfRotation);
      RobotSide sideOfFootToCropFromDrop = footDropCropper.computeSideOfFootholdToCrop(lineOfRotation);

      if (!rotationParameters.getUseCoPOccupancyGridForCropping().getValue())
         return sideOfFootToCropFromDrop;

      boolean sidesAreConsistent = sideOfFootToCropFromOccupancy != null && sideOfFootToCropFromHull != null;
      sidesAreConsistent &= sideOfFootToCropFromOccupancy == sideOfFootToCropFromHull;
      boolean sideIsntBad = sideOfFootToCropFromDrop != null && sideOfFootToCropFromDrop == sideOfFootToCropFromOccupancy;

      if (sidesAreConsistent && sideIsntBad)
         return sideOfFootToCropFromOccupancy;
      else
         return null;
   }

   public void computeShrunkenFoothold(FrameLine2DReadOnly lineOfRotation, RobotSide sideOfFootToCrop)
   {
      if (sideOfFootToCrop == null)
         return;

      // FIXME this is a work around for a bug in the cut polygon with line class that doens't work with yo frame convex polygons.
      tempPolygon.setIncludingFrame(shrunkenFootPolygon);
      shiftLineNormally(lineOfRotation, cropLine, sideOfFootToCrop.getOppositeSide(), distanceFromRotationToCrop.getValue());
      convexPolygonTools.cutPolygonWithLine(cropLine, tempPolygon, sideOfFootToCrop);
      shrunkenFootPolygon.set(tempPolygon);
   }

   private void shiftLineNormally(FrameLine2DReadOnly lineToShift, FrameLine2DBasics shiftedLineToPack, RobotSide sideToShift, double distanceToShift)
   {
      EuclidGeometryTools.perpendicularVector2D(lineToShift.getDirection(), shiftVector);
      if (sideToShift == RobotSide.RIGHT)
         shiftVector.negate();

      shiftVector.scale(distanceToShift);

      shiftedLineToPack.setIncludingFrame(lineToShift);
      shiftedLineToPack.getPoint().add(shiftVector);
   }

   public boolean shouldApplyShrunkenFoothold()
   {
      // if we are not doing partial foothold detection exit
      if (!doPartialFootholdDetection.getValue())
      {
         shrunkenFootPolygon.set(defaultFootPolygon);
         return false;
      }

      // if we shrunk the foothold too many times exit
      return shrinkCounter.getIntegerValue() < shrinkMaxLimit.getValue();
   }

   public boolean applyShrunkenFoothold(YoPlaneContactState contactStateToModify)
   {
      // make sure the foot has the right number of contact points
      controllerFootPolygon.setIncludingFrame(getShrunkenFootPolygon());
      ConvexPolygonTools.limitVerticesConservative(controllerFootPolygon, numberOfFootCornerPoints);
      controllerFootPolygonInWorld.setIncludingFrame(controllerFootPolygon);
      controllerFootPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      shrunkenFootPolygonInWorld.set(controllerFootPolygonInWorld);

      if (applyPartialFootholds.getValue())
      {
         List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();
         int i = 0;
         for (; i < controllerFootPolygon.getNumberOfVertices(); i++)
         {
            YoContactPoint contactPoint = contactPoints.get(i);
            contactPoint.set(controllerFootPolygon.getVertex(i));
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

   public FrameConvexPolygon2DReadOnly getShrunkenFootPolygon()
   {
      return shrunkenFootPolygon;
   }
}
