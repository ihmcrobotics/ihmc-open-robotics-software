package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;

public class PartialFootholdControlModule
{
   private final String name = getClass().getSimpleName();

   public enum PartialFootholdState
   {
      FULL, UNSAFE_CORNER, PARTIAL, SAFE_PARTIAL
   };

   private final YoVariableRegistry registry;

   private final EnumYoVariable<PartialFootholdState> footholdState;

   private final FootRotationCalculator footRotationCalculator;
   private final FootCoPOccupancyGrid footCoPOccupancyGrid;

   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2d defaultFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygon;
   private final FrameConvexPolygon2d unsafePolygon;
   private final YoFrameConvexPolygon2d yoUnsafePolygon;

   private final IntegerYoVariable shrinkMaxLimit;
   private final IntegerYoVariable shrinkCounter;

   private final IntegerYoVariable thresholdForCoPCellOccupancy;
   private final IntegerYoVariable thresholdForCoPRegionOccupancy;
   private final DoubleYoVariable distanceFromLineOfRotationToComputeCoPOccupancy;

   private final BooleanYoVariable doPartialFootholdDetection;

   private final FrameLine2d lineOfRotation;

   public PartialFootholdControlModule(String namePrefix, double dt, ContactablePlaneBody contactableFoot, TwistCalculator twistCalculator,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
      shrunkFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      lineOfRotation = new FrameLine2d(soleFrame);

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      footholdState = new EnumYoVariable<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2d(namePrefix + "UnsafeFootPolygon", "", ReferenceFrame.getWorldFrame(), 10, registry);

      shrinkMaxLimit = new IntegerYoVariable(namePrefix + "MaximumNumberOfFootShrink", registry);
      shrinkMaxLimit.set(1);
      shrinkCounter = new IntegerYoVariable(namePrefix + "FootShrinkCounter", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon yoGraphicPolygon = new YoArtifactPolygon(namePrefix + "UnsafeRegion", yoUnsafePolygon, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact("Partial Foothold", yoGraphicPolygon);
      }

      footRotationCalculator = new FootRotationCalculator(namePrefix, dt, contactableFoot, twistCalculator, yoGraphicsListRegistry, registry);
      footCoPOccupancyGrid = new FootCoPOccupancyGrid(namePrefix, soleFrame, walkingControllerParameters.getFootLength(), walkingControllerParameters.getFootWidth(), 20, 10, yoGraphicsListRegistry, registry);
      
      thresholdForCoPCellOccupancy = new IntegerYoVariable(namePrefix + "ThresholdForCoPCellOccupancy", registry);
      thresholdForCoPCellOccupancy.set(3);
      footCoPOccupancyGrid.setThresholdForCellActivation(thresholdForCoPCellOccupancy.getIntegerValue());

      thresholdForCoPRegionOccupancy = new IntegerYoVariable(namePrefix + "ThresholdForCoPRegionOccupancy", registry);
      thresholdForCoPRegionOccupancy.set(2);
      distanceFromLineOfRotationToComputeCoPOccupancy = new DoubleYoVariable(namePrefix + "DistanceFromLineOfRotationToComputeCoPOccupancy", registry);
      distanceFromLineOfRotationToComputeCoPOccupancy.set(0.02);
      
      doPartialFootholdDetection = new BooleanYoVariable(namePrefix + "DoPartialFootholdDetection", registry);
      doPartialFootholdDetection.set(false);
   }

   public void compute(FramePoint2d desiredCenterOfPressure, FramePoint2d centerOfPressure)
   {
      if (desiredCenterOfPressure.containsNaN() || centerOfPressure.containsNaN())
      {
         doNothing();
         return;
      }

      shrunkFootPolygon.setIncludingFrameAndUpdate(defaultFootPolygon);
      unsafePolygon.setIncludingFrameAndUpdate(defaultFootPolygon);
      footCoPOccupancyGrid.registerCenterOfPressureLocation(centerOfPressure);

      footRotationCalculator.compute(desiredCenterOfPressure, centerOfPressure);

      if (footRotationCalculator.isFootRotating())
      {
         footRotationCalculator.getLineOfRotation(lineOfRotation);

         int numberOfVerticesRemoved = ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, unsafePolygon, RobotSide.LEFT);

         if (numberOfVerticesRemoved <= 0)
         {
            doNothing();
         }
         else if (numberOfVerticesRemoved == 1)
         {
            footholdState.set(PartialFootholdState.UNSAFE_CORNER);
            computeShrunkFoothold(desiredCenterOfPressure);
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

   private void doNothing()
   {
      footholdState.set(PartialFootholdState.FULL);
      yoUnsafePolygon.hide();
   }

   private void computeShrunkFoothold(FramePoint2d desiredCenterOfPressure)
   {
      int numberOfCellsOccupiedOnSideOfLine = footCoPOccupancyGrid.computeNumberOfCellsOccupiedOnSideOfLine(lineOfRotation, RobotSide.RIGHT, distanceFromLineOfRotationToComputeCoPOccupancy.getDoubleValue());
      boolean wasCoPInThatRegion = numberOfCellsOccupiedOnSideOfLine >= thresholdForCoPRegionOccupancy.getIntegerValue();
      if (unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3) && !wasCoPInThatRegion)
      {
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
         return false;

      if (footholdState.getEnumValue() != PartialFootholdState.PARTIAL)
         return false;

      if (shrinkCounter.getIntegerValue() >= shrinkMaxLimit.getIntegerValue())
         return false;

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();

      if (contactPoints.size() != shrunkFootPolygon.getNumberOfVertices())
         return false;

      for (int i = 0; i < shrunkFootPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2d vertex = shrunkFootPolygon.getFrameVertex(i);
         tempPosition.setIncludingFrame(vertex.getReferenceFrame(), vertex.getX(), vertex.getY(), 0.0);
         contactPoints.get(i).setPosition(tempPosition);
      }

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
   }
}
