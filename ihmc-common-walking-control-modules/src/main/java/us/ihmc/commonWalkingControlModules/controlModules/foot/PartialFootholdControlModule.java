package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class PartialFootholdControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();

   public enum PartialFootholdState
   {
      FULL, PARTIAL
   };

   private final YoVariableRegistry registry;

   private final YoEnum<PartialFootholdState> footholdState;

   public enum RotationCalculatorType
   {
      VELOCITY, GEOMETRY, BOTH;
      public static RotationCalculatorType[] values = values();
   }
   private final EnumMap<RotationCalculatorType, FootRotationCalculator> rotationCalculators = new EnumMap<>(RotationCalculatorType.class);
   private final EnumMap<RotationCalculatorType, FrameLine2D> lineOfRotations = new EnumMap<>(RotationCalculatorType.class);
   private final YoEnum<RotationCalculatorType> rotationCalculatorType;

   private final RotationVerificator rotationVerificator;

   private final FootCoPOccupancyGrid footCoPOccupancyGrid;

   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2D defaultFootPolygon;
   private final FrameConvexPolygon2D shrunkFootPolygon;
   private final FrameConvexPolygon2D shrunkFootPolygonInWorld;
   private final YoFrameConvexPolygon2D yoShrunkFootPolygon;
   private final FrameConvexPolygon2D controllerFootPolygon;
   private final FrameConvexPolygon2D controllerFootPolygonInWorld;
   private final FrameConvexPolygon2D backupFootPolygon;
   private final FrameConvexPolygon2D unsafePolygon;
   private final YoFrameConvexPolygon2D yoUnsafePolygon;

   private final FrameConvexPolygon2D fullSupportAfterShrinking = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoFullSupportAfterShrinking;

   private final YoInteger shrinkMaxLimit;
   private final YoInteger shrinkCounter;

   private final YoInteger numberOfCellsOccupiedOnSideOfLine;

   private final YoInteger thresholdForCoPRegionOccupancy;
   private final YoDouble distanceFromLineOfRotationToComputeCoPOccupancy;

   private final YoBoolean doPartialFootholdDetection;

   private final FrameLine2D lineOfRotation;

   private final YoBoolean useCoPOccupancyGrid;
   private final YoBoolean cropToConvexHullOfCoPs;
   private final YoBoolean fitLineToCoPs;

   private final int footCornerPoints;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final FramePoint2D capturePoint = new FramePoint2D();
   private RobotSide robotSide;

   /**
    * Variables for checking the area of the unsafe part of the foothold.
    */
   private final YoDouble unsafeArea;
   private final YoDouble minAreaToConsider;
   private final YoBoolean unsafeAreaAboveThreshold;

   private final YoBoolean expectingLineContact;
   private final FramePoint2D dummyDesiredCop = new FramePoint2D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public PartialFootholdControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                       WalkingControllerParameters walkingControllerParameters, ExplorationParameters explorationParameters,
                                       YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ContactableFoot contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      String namePrefix = contactableFoot.getRigidBody().getName();
      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      footCornerPoints = contactableFoot.getTotalNumberOfContactPoints();
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
      shrunkFootPolygon = new FrameConvexPolygon2D(defaultFootPolygon);
      shrunkFootPolygonInWorld = new FrameConvexPolygon2D(defaultFootPolygon);
      controllerFootPolygon = new FrameConvexPolygon2D(defaultFootPolygon);
      controllerFootPolygonInWorld = new FrameConvexPolygon2D(defaultFootPolygon);
      backupFootPolygon = new FrameConvexPolygon2D(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2D(defaultFootPolygon);
      lineOfRotation = new FrameLine2D(soleFrame);

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      footholdState = new YoEnum<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2D(namePrefix + "UnsafeFootPolygon", "", worldFrame, 10, registry);
      yoShrunkFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "ShrunkFootPolygon", "", worldFrame, 20, registry);
      yoFullSupportAfterShrinking = new YoFrameConvexPolygon2D(namePrefix + "FullSupportAfterShrinking", "", worldFrame, 20, registry);

      shrinkCounter = new YoInteger(namePrefix + "ShrinkCounter", registry);

      numberOfCellsOccupiedOnSideOfLine = new YoInteger(namePrefix + "NumberOfCellsOccupiedOnSideOfLine", registry);

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();

         YoArtifactPolygon yoGraphicPolygon = new YoArtifactPolygon(namePrefix + "UnsafeRegion", yoUnsafePolygon, Color.RED, false);
         yoGraphicPolygon.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(listName, yoGraphicPolygon);

         YoArtifactPolygon yoShrunkPolygon = new YoArtifactPolygon(namePrefix + "ShrunkPolygon", yoShrunkFootPolygon, Color.CYAN, false);
         yoShrunkPolygon.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(listName, yoShrunkPolygon);
      }

      footCoPOccupancyGrid = new FootCoPOccupancyGrid(namePrefix, soleFrame, 40, 20, walkingControllerParameters, explorationParameters, yoGraphicsListRegistry,
                                                      registry);

      shrinkMaxLimit = explorationParameters.getShrinkMaxLimit();
      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();
      useCoPOccupancyGrid = explorationParameters.getUseCopOccupancyGrid();
      rotationCalculatorType = explorationParameters.getRotationCalculatorType();
      minAreaToConsider = explorationParameters.getMinAreaToConsider();

      doPartialFootholdDetection = new YoBoolean(namePrefix + "DoPartialFootholdDetection", registry);
      doPartialFootholdDetection.set(false);

      cropToConvexHullOfCoPs = new YoBoolean(namePrefix + "CropToConvexHullOfCoPs", registry);
      cropToConvexHullOfCoPs.set(false);

      fitLineToCoPs = new YoBoolean(namePrefix + "FitLineToCoPs", registry);
      fitLineToCoPs.set(false);

      expectingLineContact = new YoBoolean(namePrefix + "ExpectingLineContact", registry);
      expectingLineContact.set(false);

      double dt = controllerToolbox.getControlDT();

      FootRotationCalculator velocityFootRotationCalculator =
            new VelocityFootRotationCalculator(namePrefix, dt, contactableFoot, explorationParameters, yoGraphicsListRegistry, registry);
      FootRotationCalculator geometricFootRotationCalculator =
            new GeometricFootRotationCalculator(namePrefix, contactableFoot, explorationParameters, yoGraphicsListRegistry, registry);
      rotationCalculators.put(RotationCalculatorType.VELOCITY, velocityFootRotationCalculator);
      rotationCalculators.put(RotationCalculatorType.GEOMETRY, geometricFootRotationCalculator);
      lineOfRotations.put(RotationCalculatorType.VELOCITY, new FrameLine2D(soleFrame));
      lineOfRotations.put(RotationCalculatorType.GEOMETRY, new FrameLine2D(soleFrame));

      rotationVerificator = new RotationVerificator(namePrefix, contactableFoot, explorationParameters, registry);

      unsafeArea = new YoDouble(namePrefix + "UnsafeArea", registry);
      unsafeAreaAboveThreshold = new YoBoolean(namePrefix + "UnsafeAreaAboveThreshold", registry);
   }

   public void compute(FramePoint2D desiredCenterOfPressure, FramePoint2D centerOfPressure)
   {
      footCoPOccupancyGrid.update();

      if (desiredCenterOfPressure.containsNaN() || centerOfPressure.containsNaN())
      {
         doNothing();
         return;
      }

      unsafePolygon.setIncludingFrame(shrunkFootPolygon);
      footCoPOccupancyGrid.registerCenterOfPressureLocation(centerOfPressure);

      boolean atLeastOneTriggered = false;
      for (RotationCalculatorType calculatorType : RotationCalculatorType.values)
      {
         if (!rotationCalculators.containsKey(calculatorType)) continue;
         rotationCalculators.get(calculatorType).compute(desiredCenterOfPressure, centerOfPressure);
         rotationCalculators.get(calculatorType).getLineOfRotation(lineOfRotations.get(calculatorType));

         if (rotationCalculators.get(calculatorType).isFootRotating())
         {
            boolean verified = rotationVerificator.isRotating(centerOfPressure, desiredCenterOfPressure, lineOfRotations.get(calculatorType));
            atLeastOneTriggered = atLeastOneTriggered || verified;
            if (verified)
            {
               lineOfRotation.setIncludingFrame(lineOfRotations.get(calculatorType));
            }
         }
      }

      boolean triggerCutting;
      if (rotationCalculatorType.getEnumValue() == RotationCalculatorType.BOTH)
      {
         triggerCutting = atLeastOneTriggered;
      }
      else
      {
         FootRotationCalculator activeCalculator = rotationCalculators.get(rotationCalculatorType.getEnumValue());
         activeCalculator.getLineOfRotation(lineOfRotation);
         boolean verified = rotationVerificator.isRotating(centerOfPressure, desiredCenterOfPressure, lineOfRotation);
         triggerCutting = activeCalculator.isFootRotating() && verified;
      }

      if (triggerCutting)
      {
         footholdState.set(PartialFootholdState.PARTIAL);
         computeShrunkFoothold(desiredCenterOfPressure);

         if (expectingLineContact.getBooleanValue())
         {
            lineOfRotation.shiftToLeft(0.01);
            lineOfRotation.getDirection().negate();

            dummyDesiredCop.setToNaN(unsafePolygon.getReferenceFrame());
            computeShrunkFoothold(dummyDesiredCop);
         }
      }
      else
      {
         doNothing();
      }

   }

   public void getShrunkPolygonCentroid(FramePoint2D centroidToPack)
   {
      centroidToPack.setIncludingFrame(shrunkFootPolygon.getCentroid());
   }

   private void doNothing()
   {
      footholdState.set(PartialFootholdState.FULL);
      yoUnsafePolygon.clear();
      shrunkFootPolygonInWorld.setIncludingFrame(shrunkFootPolygon);
      shrunkFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
      yoShrunkFootPolygon.set(shrunkFootPolygonInWorld);
      unsafeArea.set(0.0);
   }

   private void computeShrunkFoothold(FramePoint2D desiredCenterOfPressure)
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

      boolean desiredCopInPolygon = desiredCenterOfPressure.containsNaN() || unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3);

      if (desiredCopInPolygon && !wasCoPInThatRegion && areaBigEnough)
      {
         backupFootPolygon.set(shrunkFootPolygon);
         convexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkFootPolygon, RobotSide.RIGHT);
         unsafePolygon.changeFrameAndProjectToXYPlane(worldFrame);
         yoUnsafePolygon.set(unsafePolygon);

         shrunkFootPolygonInWorld.setIncludingFrame(shrunkFootPolygon);
         shrunkFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);
         yoShrunkFootPolygon.set(shrunkFootPolygonInWorld);
      }
      else
      {
         doNothing();
      }
   }

   private final FramePoint3D tempPosition = new FramePoint3D();

   public boolean applyShrunkPolygon(YoPlaneContactState contactStateToModify)
   {
      if (cropToConvexHullOfCoPs.getBooleanValue())
      {
         footCoPOccupancyGrid.computeConvexHull(shrunkFootPolygon);
         cropToConvexHullOfCoPs.set(false);
      }
      else if (fitLineToCoPs.getBooleanValue())
      {
         fitLine();
         fitLineToCoPs.set(false);
      }
      else
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
      }

      // make sure the foot has the right number of contact points
      controllerFootPolygon.setIncludingFrame(shrunkFootPolygon);
      ConvexPolygonTools.limitVerticesConservative(controllerFootPolygon, footCornerPoints);
      controllerFootPolygonInWorld.setIncludingFrame(controllerFootPolygon);
      controllerFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      // if the icp is in the area that would be cut off exit
      FrameConvexPolygon2DReadOnly oppositeFootPolygon = controllerToolbox.getBipedSupportPolygons().getFootPolygonInWorldFrame(robotSide.getOppositeSide());
      fullSupportAfterShrinking.setIncludingFrame(oppositeFootPolygon);
      fullSupportAfterShrinking.changeFrameAndProjectToXYPlane(worldFrame);
      fullSupportAfterShrinking.addVertices(controllerFootPolygonInWorld);
      fullSupportAfterShrinking.update();
      controllerToolbox.getCapturePoint(capturePoint);
      yoFullSupportAfterShrinking.set(fullSupportAfterShrinking);
//      boolean icpInPolygon = fullSupportAfterShrinking.isPointInside(capturePoint);
//      if (!icpInPolygon)
//      {
//         shrunkFootPolygon.set(backupFootPolygon);
//         return false;
//      }

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();
      for (int i = 0; i < controllerFootPolygon.getNumberOfVertices(); i++)
      {
         tempPosition.setIncludingFrame(controllerFootPolygon.getVertex(i), 0.0);
         YoContactPoint contactPoint = contactPoints.get(i);
         contactPoint.setPosition(tempPosition);
         contactPoint.setInContact(true);
      }

      for (int i = controllerFootPolygon.getNumberOfVertices(); i < contactPoints.size(); i++)
      {
         contactPoints.get(i).setInContact(false);
      }

      backupFootPolygon.set(shrunkFootPolygon);
      shrinkCounter.increment();
      return true;
   }

   private final FrameLine2D line = new FrameLine2D();
   private final FrameLine2D lineL = new FrameLine2D();
   private final FrameLine2D lineR = new FrameLine2D();
   private static final double width = 0.01;
   private void fitLine()
   {
      if (!footCoPOccupancyGrid.fitLineToData(line))
         return;

      lineL.setIncludingFrame(line);
      lineL.shiftToLeft(width/2.0);

      lineR.setIncludingFrame(line);
      lineR.shiftToRight(width/2.0);

      backupFootPolygon.set(shrunkFootPolygon);
      shrunkFootPolygon.clear();
      shrunkFootPolygon.addVertices(FrameVertex2DSupplier.asFrameVertex2DSupplier(defaultFootPolygon.intersectionWith(lineL)));
      shrunkFootPolygon.addVertices(FrameVertex2DSupplier.asFrameVertex2DSupplier(defaultFootPolygon.intersectionWith(lineR)));
      shrunkFootPolygon.update();
   }

   public void requestLineFit()
   {
      fitLineToCoPs.set(true);
   }

   public void reset()
   {
      shrinkCounter.set(0);
      footholdState.set(null);
      yoUnsafePolygon.clear();
      yoShrunkFootPolygon.clear();
      yoFullSupportAfterShrinking.clear();
      for (RotationCalculatorType calculatorType : RotationCalculatorType.values)
      {
         if (!rotationCalculators.containsKey(calculatorType)) continue;
         rotationCalculators.get(calculatorType).reset();
      }
      footCoPOccupancyGrid.reset();
      shrunkFootPolygon.setIncludingFrame(defaultFootPolygon);
      backupFootPolygon.setIncludingFrame(defaultFootPolygon);
   }

   public void projectOntoShrunkenPolygon(FramePoint2D pointToProject)
   {
      shrunkFootPolygon.orthogonalProjection(pointToProject);
   }

   public void getSupportPolygon(FrameConvexPolygon2D polygonToPack)
   {
      polygonToPack.setIncludingFrame(shrunkFootPolygon);
   }

   public void clearCoPGrid()
   {
      footCoPOccupancyGrid.reset();
   }

   public void turnOffCropping()
   {
      doPartialFootholdDetection.set(false);
   }

   public void turnOnCropping()
   {
      doPartialFootholdDetection.set(true);
   }

   public void informExplorationDone()
   {
      if (expectingLineContact.getBooleanValue())
      {
         requestLineFit();
         turnOffCropping();
      }
   }
}
