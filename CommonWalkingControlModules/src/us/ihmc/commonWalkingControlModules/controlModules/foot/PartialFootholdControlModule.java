package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
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
      VELOCITY, GEOMETRY, BOTH;
      public static RotationCalculatorType[] values = values();
   }
   private final EnumMap<RotationCalculatorType, FootRotationCalculator> rotationCalculators = new EnumMap<>(RotationCalculatorType.class);
   private final EnumMap<RotationCalculatorType, FrameLine2d> lineOfRotations = new EnumMap<>(RotationCalculatorType.class);
   private final EnumYoVariable<RotationCalculatorType> rotationCalculatorType;

   private final RotationVerificator rotationVerificator;

   private final FootCoPOccupancyGrid footCoPOccupancyGrid;

   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2d defaultFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygonInWorld;
   private final YoFrameConvexPolygon2d yoShrunkFootPolygon;
   private final FrameConvexPolygon2d controllerFootPolygon;
   private final FrameConvexPolygon2d controllerFootPolygonInWorld;
   private final FrameConvexPolygon2d backupFootPolygon;
   private final FrameConvexPolygon2d unsafePolygon;
   private final YoFrameConvexPolygon2d yoUnsafePolygon;

   private final FrameConvexPolygon2d fullSupportAfterShrinking = new FrameConvexPolygon2d();
   private final YoFrameConvexPolygon2d yoFullSupportAfterShrinking;

   private final IntegerYoVariable shrinkMaxLimit;
   private final IntegerYoVariable shrinkCounter;

   private final IntegerYoVariable numberOfCellsOccupiedOnSideOfLine;

   private final IntegerYoVariable thresholdForCoPRegionOccupancy;
   private final DoubleYoVariable distanceFromLineOfRotationToComputeCoPOccupancy;

   private final BooleanYoVariable doPartialFootholdDetection;

   private final FrameLine2d lineOfRotation;

   private final BooleanYoVariable useCoPOccupancyGrid;
   private final BooleanYoVariable cropToConvexHullOfCoPs;
   private final BooleanYoVariable fitLineToCoPs;

   private final int footCornerPoints;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final FramePoint2d capturePoint = new FramePoint2d();
   private RobotSide robotSide;

   /**
    * Variables for checking the area of the unsafe part of the foothold.
    */
   private final DoubleYoVariable unsafeArea;
   private final DoubleYoVariable minAreaToConsider;
   private final BooleanYoVariable unsafeAreaAboveThreshold;

   private final BooleanYoVariable expectingLineContact;
   private final FramePoint2d dummyDesiredCop = new FramePoint2d();

   public PartialFootholdControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ContactableFoot contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      String namePrefix = contactableFoot.getRigidBody().getName();
      this.controllerToolbox = momentumBasedController;
      this.robotSide = robotSide;

      footCornerPoints = contactableFoot.getTotalNumberOfContactPoints();
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
      shrunkFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      shrunkFootPolygonInWorld = new FrameConvexPolygon2d(defaultFootPolygon);
      controllerFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      controllerFootPolygonInWorld = new FrameConvexPolygon2d(defaultFootPolygon);
      backupFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      lineOfRotation = new FrameLine2d(soleFrame);

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);
      ExplorationParameters explorationParameters = walkingControllerParameters.getOrCreateExplorationParameters(registry);

      footholdState = new EnumYoVariable<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2d(namePrefix + "UnsafeFootPolygon", "", worldFrame, 10, registry);
      yoShrunkFootPolygon = new YoFrameConvexPolygon2d(namePrefix + "ShrunkFootPolygon", "", worldFrame, 20, registry);
      yoFullSupportAfterShrinking = new YoFrameConvexPolygon2d(namePrefix + "FullSupportAfterShrinking", "", worldFrame, 20, registry);

      shrinkCounter = new IntegerYoVariable(namePrefix + "ShrinkCounter", registry);

      numberOfCellsOccupiedOnSideOfLine = new IntegerYoVariable(namePrefix + "NumberOfCellsOccupiedOnSideOfLine", registry);

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

      footCoPOccupancyGrid = new FootCoPOccupancyGrid(namePrefix, soleFrame, 40, 20, walkingControllerParameters, yoGraphicsListRegistry, registry);

      shrinkMaxLimit = explorationParameters.getShrinkMaxLimit();
      thresholdForCoPRegionOccupancy = explorationParameters.getThresholdForCoPRegionOccupancy();
      distanceFromLineOfRotationToComputeCoPOccupancy = explorationParameters.getDistanceFromLineOfRotationToComputeCoPOccupancy();
      useCoPOccupancyGrid = explorationParameters.getUseCopOccupancyGrid();
      rotationCalculatorType = explorationParameters.getRotationCalculatorType();
      minAreaToConsider = explorationParameters.getMinAreaToConsider();

      doPartialFootholdDetection = new BooleanYoVariable(namePrefix + "DoPartialFootholdDetection", registry);
      doPartialFootholdDetection.set(false);

      cropToConvexHullOfCoPs = new BooleanYoVariable(namePrefix + "CropToConvexHullOfCoPs", registry);
      cropToConvexHullOfCoPs.set(false);

      fitLineToCoPs = new BooleanYoVariable(namePrefix + "FitLineToCoPs", registry);
      fitLineToCoPs.set(false);

      expectingLineContact = new BooleanYoVariable(namePrefix + "ExpectingLineContact", registry);
      expectingLineContact.set(false);

      double dt = momentumBasedController.getControlDT();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();

      FootRotationCalculator velocityFootRotationCalculator =
            new VelocityFootRotationCalculator(namePrefix, dt, contactableFoot, twistCalculator, explorationParameters, yoGraphicsListRegistry, registry);
      FootRotationCalculator geometricFootRotationCalculator =
            new GeometricFootRotationCalculator(namePrefix, contactableFoot, explorationParameters, yoGraphicsListRegistry, registry);
      rotationCalculators.put(RotationCalculatorType.VELOCITY, velocityFootRotationCalculator);
      rotationCalculators.put(RotationCalculatorType.GEOMETRY, geometricFootRotationCalculator);
      lineOfRotations.put(RotationCalculatorType.VELOCITY, new FrameLine2d(soleFrame));
      lineOfRotations.put(RotationCalculatorType.GEOMETRY, new FrameLine2d(soleFrame));

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
            lineOfRotation.negateDirection();

            dummyDesiredCop.setToNaN(unsafePolygon.getReferenceFrame());
            computeShrunkFoothold(dummyDesiredCop);
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

      boolean desiredCopInPolygon = unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3);

      if (desiredCopInPolygon && !wasCoPInThatRegion && areaBigEnough)
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
      controllerFootPolygonInWorld.setIncludingFrameAndUpdate(controllerFootPolygon);
      controllerFootPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      // if the icp is in the area that would be cut off exit
      FrameConvexPolygon2d oppositeFootPolygon = controllerToolbox.getBipedSupportPolygons().getFootPolygonInWorldFrame(robotSide.getOppositeSide());
      fullSupportAfterShrinking.setIncludingFrameAndUpdate(oppositeFootPolygon);
      fullSupportAfterShrinking.changeFrameAndProjectToXYPlane(worldFrame);
      fullSupportAfterShrinking.addVertices(controllerFootPolygonInWorld);
      fullSupportAfterShrinking.update();
      controllerToolbox.getCapturePoint(capturePoint);
      yoFullSupportAfterShrinking.setFrameConvexPolygon2d(fullSupportAfterShrinking);
//      boolean icpInPolygon = fullSupportAfterShrinking.isPointInside(capturePoint);
//      if (!icpInPolygon)
//      {
//         shrunkFootPolygon.set(backupFootPolygon);
//         return false;
//      }

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();
      for (int i = 0; i < controllerFootPolygon.getNumberOfVertices(); i++)
      {
         controllerFootPolygon.getFrameVertexXY(i, tempPosition);
         contactPoints.get(i).setPosition(tempPosition);
      }

      backupFootPolygon.set(shrunkFootPolygon);
      shrinkCounter.increment();
      return true;
   }

   private final FrameLine2d line = new FrameLine2d();
   private final FrameLine2d lineL = new FrameLine2d();
   private final FrameLine2d lineR = new FrameLine2d();
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
      shrunkFootPolygon.addVertices(defaultFootPolygon.intersectionWith(lineL));
      shrunkFootPolygon.addVertices(defaultFootPolygon.intersectionWith(lineR));
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
      yoUnsafePolygon.hide();
      yoShrunkFootPolygon.hide();
      yoFullSupportAfterShrinking.hide();
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
