package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class MultiStepPushRecoveryCalculatorVisualizer implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<YoFramePoint2D> capturePointsAtTouchdown = new ArrayList<>();
   private final List<YoFramePoint2D> recoveryStepLocations = new ArrayList<>();

   private final List<YoFrameConvexPolygon2D> yoCaptureRegionsAtTouchdown = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoIntersectingRegions = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoReachableRegions = new ArrayList<>();

   private final int maxRegionDepth;

   private final String suffix;

   public MultiStepPushRecoveryCalculatorVisualizer(String suffix, int maxRegionDepth, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.suffix = suffix;
      this.maxRegionDepth = maxRegionDepth;
      if (graphicsListRegistry != null)
      {
         YoRegistry registry = new YoRegistry(getClass().getSimpleName());

         String listName = getClass().getSimpleName();
         for (int i = 0; i < maxRegionDepth; i++)
         {
            String captureName = "captureRegion" + i;

            capturePointsAtTouchdown.add(new YoFramePoint2D("capturePointAtTouchdown" + i, worldFrame, registry));
            recoveryStepLocations.add(new YoFramePoint2D("recoveryStepLocation" + i, worldFrame, registry));

            YoFrameConvexPolygon2D yoCaptureRegionPolygonAtTouchdown = new YoFrameConvexPolygon2D(captureName + "AtTouchdown",
                                                                                                  suffix,
                                                                                                  worldFrame,
                                                                                                  30,
                                                                                                  registry);
            yoCaptureRegionsAtTouchdown.add(yoCaptureRegionPolygonAtTouchdown);

            YoArtifactPolygon capturePolygonAtTouchdownArtifact = new YoArtifactPolygon(captureName + "AtTouchdown" + suffix,
                                                                                        yoCaptureRegionPolygonAtTouchdown,
                                                                                        Color.RED,
                                                                                        false,
                                                                                        true);

            String reachableName = "reachableRegion" + i;

            YoFrameConvexPolygon2D yoReachableRegionPolygon = new YoFrameConvexPolygon2D(reachableName, suffix, worldFrame, 30, registry);
            yoReachableRegions.add(yoReachableRegionPolygon);

            YoArtifactPolygon reachablePolygonArtifact = new YoArtifactPolygon(reachableName + suffix, yoReachableRegionPolygon, Color.BLUE, false);

            String intersectingName = "intersectingRegion" + i;

            YoFrameConvexPolygon2D yoIntersectingRegion = new YoFrameConvexPolygon2D(intersectingName, suffix, worldFrame, 30, registry);
            yoIntersectingRegions.add(yoIntersectingRegion);

            YoArtifactPolygon intersectingPolygonArtifact = new YoArtifactPolygon(intersectingName + suffix, yoIntersectingRegion, Color.YELLOW, false);

            YoGraphicPosition touchdownICPViz = new YoGraphicPosition("capturePointTouchdown" + i + suffix,
                                                                      capturePointsAtTouchdown.get(i),
                                                                      0.01,
                                                                      YoAppearance.Yellow(),
                                                                      YoGraphicPosition.GraphicType.SOLID_BALL);
            YoGraphicPosition footstepViz = new YoGraphicPosition("recoveryStepLocation" + i + suffix,
                                                                  recoveryStepLocations.get(i),
                                                                  0.01,
                                                                  YoAppearance.Blue(),
                                                                  YoGraphicPosition.GraphicType.SOLID_BALL);

            graphicsListRegistry.registerArtifact(listName, capturePolygonAtTouchdownArtifact);
            graphicsListRegistry.registerArtifact(listName, reachablePolygonArtifact);
            graphicsListRegistry.registerArtifact(listName, intersectingPolygonArtifact);
            graphicsListRegistry.registerArtifact(listName, touchdownICPViz.createArtifact());
            graphicsListRegistry.registerArtifact(listName, footstepViz.createArtifact());
         }

         parentRegistry.addChild(registry);

         reset();
      }
   }

   public void reset()
   {
      for (int i = 0; i < maxRegionDepth; i++)
      {
         capturePointsAtTouchdown.get(i).setToNaN();
         recoveryStepLocations.get(i).setToNaN();

         yoCaptureRegionsAtTouchdown.get(i).setToNaN();
         yoReachableRegions.get(i).setToNaN();
         yoIntersectingRegions.get(i).setToNaN();
      }
   }

   public void visualize(MultiStepPushRecoveryCalculator pushRecoveryCalculator)
   {
      int depth = Math.min(pushRecoveryCalculator.getNumberOfRecoverySteps(), maxRegionDepth);

      for (int i = 0; i < depth; i++)
      {
         capturePointsAtTouchdown.get(i).set(pushRecoveryCalculator.getCapturePointAtTouchdown(i));
         recoveryStepLocations.get(i).set(pushRecoveryCalculator.getRecoveryStepLocation(i));
         yoReachableRegions.get(i).set(pushRecoveryCalculator.getReachableRegion(i));
         yoCaptureRegionsAtTouchdown.get(i).set(pushRecoveryCalculator.getCaptureRegionAtTouchdown(i));
         yoIntersectingRegions.get(i).set(pushRecoveryCalculator.getIntersectingRegion(i));
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < maxRegionDepth; i++)
      {
         String captureName = "captureRegion" + i;
         // TODO The stroke graphic was initially dashed
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(captureName + "AtTouchdown" + suffix,
                                                                         yoCaptureRegionsAtTouchdown.get(i),
                                                                         ColorDefinitions.Red()));

         String reachableName = "reachableRegion" + i;
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(reachableName + suffix, yoReachableRegions.get(i), ColorDefinitions.Blue()));

         String intersectingName = "intersectingRegion" + i;
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(intersectingName + suffix, yoIntersectingRegions.get(i), ColorDefinitions.Yellow()));

         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("capturePointTouchdown" + i + suffix,
                                                                       capturePointsAtTouchdown.get(i),
                                                                       0.02,
                                                                       ColorDefinitions.Yellow(),
                                                                       DefaultPoint2DGraphic.CIRCLE_FILLED));
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("recoveryStepLocation" + i + suffix,
                                                                       recoveryStepLocations.get(i),
                                                                       0.02,
                                                                       ColorDefinitions.Blue(),
                                                                       DefaultPoint2DGraphic.CIRCLE_FILLED));
      }
      return group;
   }
}
