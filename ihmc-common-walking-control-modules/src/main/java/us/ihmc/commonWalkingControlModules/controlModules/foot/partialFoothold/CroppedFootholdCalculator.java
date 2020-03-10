package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CroppedFootholdCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FrameConvexPolygon2D defaultFootPolygon;
   private final FrameConvexPolygon2D shrunkenFootPolygon;
   
   private final YoDouble minAreaToConsider;
   private final YoBoolean hasEnoughAreaToCrop;

   private final FootCoPOccupancyCropper footCoPOccupancyGrid;
   private final FootCoPHullCropper footCoPHullCropper;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public CroppedFootholdCalculator(String namePrefix, ReferenceFrame soleFrame, ContactableFoot contactableFoot,
                                    WalkingControllerParameters walkingControllerParameters,
                                    ExplorationParameters explorationParameters, YoVariableRegistry parentRegistry,
                                    YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
      shrunkenFootPolygon = new FrameConvexPolygon2D(defaultFootPolygon);

      footCoPOccupancyGrid = new FootCoPOccupancyCropper(namePrefix, soleFrame, 40, 20, walkingControllerParameters, explorationParameters, yoGraphicsListRegistry,
                                                         registry);
      footCoPHullCropper = new FootCoPHullCropper(namePrefix, soleFrame, 40, 20, walkingControllerParameters, explorationParameters, yoGraphicsListRegistry,
                                                         registry);

      hasEnoughAreaToCrop = new YoBoolean(namePrefix + "HasEnoughAreaToCrop", registry);

      minAreaToConsider = explorationParameters.getMinAreaToConsider();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      reset(defaultFootPolygon);
   }

   public void reset(FrameConvexPolygon2DReadOnly polygon)
   {
      shrunkenFootPolygon.set(polygon);
   }

   public void update(FramePoint2DReadOnly measuredCoP)
   {
      footCoPOccupancyGrid.update();
      footCoPHullCropper.update();

      if (measuredCoP.containsNaN())
         return;

      footCoPOccupancyGrid.registerCenterOfPressureLocation(measuredCoP);
      footCoPHullCropper.registerCenterOfPressureLocation(measuredCoP);
   }

   public void computeShrunkenFoothold(FrameLine2DReadOnly lineOfRotation)
   {
      RobotSide sideOfFootToCropFromOccupancy = footCoPOccupancyGrid.computeSideOfFootholdToCrop(lineOfRotation);
      RobotSide sideOfFootToCropFromHull = footCoPHullCropper.computeSideOfFootholdToCrop(lineOfRotation);

      boolean sidesAreConsistent = sideOfFootToCropFromOccupancy != null && sideOfFootToCropFromOccupancy == sideOfFootToCropFromHull;
      hasEnoughAreaToCrop.set(shrunkenFootPolygon.getArea() > minAreaToConsider.getDoubleValue());

      if (sidesAreConsistent && hasEnoughAreaToCrop.getBooleanValue())
      {
         convexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkenFootPolygon, sideOfFootToCropFromOccupancy);
      }
   }
}
