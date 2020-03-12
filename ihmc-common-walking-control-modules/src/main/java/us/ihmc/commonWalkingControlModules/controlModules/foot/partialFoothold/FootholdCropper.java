package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.List;

public class FootholdCropper
{
   private final FrameConvexPolygon2D defaultFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygon;
   private final YoFrameConvexPolygon2D shrunkenFootPolygonInWorld;
   private final FrameConvexPolygon2D controllerFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D controllerFootPolygonInWorld = new FrameConvexPolygon2D();

   private final DoubleProvider minAreaToConsider;
   private final YoBoolean hasEnoughAreaToCrop;

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

   public FootholdCropper(String namePrefix,
                          ContactableFoot contactableFoot,
                          FootholdRotationParameters rotationParameters,
                          YoVariableRegistry parentRegistry,
                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
      numberOfFootCornerPoints = contactableFoot.getTotalNumberOfContactPoints();


      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      verifier = new CropVerifier(namePrefix, contactableFoot.getSoleFrame(), 0.02, 0.02, rotationParameters, registry);

      shrunkenFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygon", "", soleFrame, 20, registry);
      shrunkenFootPolygonInWorld = new YoFrameConvexPolygon2D(namePrefix + "ShrunkenFootPolygonInWorld", "", ReferenceFrame.getWorldFrame(), 20, registry);
      shrunkenFootPolygon.set(defaultFootPolygon);

      shouldShrinkFoothold = new YoBoolean(namePrefix + "ShouldShrinkFoothold", registry);

      double resolution = 0.05;
      footCoPOccupancyGrid = new FootCoPOccupancyCalculator(namePrefix, soleFrame, resolution, resolution, rotationParameters, null, registry);
      footCoPHullCropper = new FootCoPHullCalculator(namePrefix, soleFrame, resolution, resolution, null, registry);
      sideOfFootToCrop = new YoEnum<>(namePrefix + "SideOfFootToCrop", registry, RobotSide.class, true);

      hasEnoughAreaToCrop = new YoBoolean(namePrefix + "HasEnoughAreaToCrop", registry);

      minAreaToConsider = rotationParameters.getMinimumAreaForCropping();

      doPartialFootholdDetection = new YoBoolean(namePrefix + "DoPartialFootholdDetection", registry);
      applyPartialFootholds = new YoBoolean(namePrefix + "ApplyPartialFootholds", registry);
      doPartialFootholdDetection.set(true);
      applyPartialFootholds.set(true);
      shrinkCounter = new YoInteger(namePrefix + "ShrinkCounter", registry);
      shrinkMaxLimit = rotationParameters.getShrinkMaxLimit();

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();

         YoArtifactPolygon yoShrunkPolygon = new YoArtifactPolygon(namePrefix + "ShrunkPolygon", shrunkenFootPolygonInWorld, Color.BLUE, false);
         yoShrunkPolygon.setVisible(true);
         yoGraphicsListRegistry.registerArtifact(listName, yoShrunkPolygon);
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

      footCoPHullCropper.reset();
      footCoPOccupancyGrid.reset();
   }

   public void update(FramePoint2DReadOnly measuredCoP, FramePoint2DReadOnly desiredCoP)
   {
      shouldShrinkFoothold.set(false);
      footCoPOccupancyGrid.update();
      footCoPHullCropper.update();
      verifier.update(desiredCoP);

      if (measuredCoP.containsNaN())
         return;

      footCoPOccupancyGrid.registerCenterOfPressureLocation(measuredCoP);
      footCoPHullCropper.registerCenterOfPressureLocation(measuredCoP);
   }

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
            convexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkenFootPolygon, sideOfFootToCrop.getEnumValue().getOppositeSide());
      }
      else
      {
         shouldShrinkFoothold.set(false);
         sideOfFootToCrop.set(null);
      }
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
