package us.ihmc.avatar.stepAdjustment;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class PlanarRegionSnapVisualizer implements PlanarRegionSnapperCallback
{
   private static final int numberOfStepsToVisualize = 3;
   private static final int maximumVertices = 40;

   private final FootholdData[] footholdData = new FootholdData[numberOfStepsToVisualize];

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger stepsVisualized = new YoInteger("stepsVisualized", registry);

   public PlanarRegionSnapVisualizer(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      for (int i = 0; i < numberOfStepsToVisualize; i++)
         footholdData[i] = new FootholdData("" + i, registry, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      stepsVisualized.set(-1);

      for (int i = 0; i < numberOfStepsToVisualize; i++)
         footholdData[i].reset();
   }

   @Override
   public void advanceFootIndex()
   {
      stepsVisualized.increment();
   }

   @Override
   public void recordUnadjustedFootstep(FramePose3DReadOnly footPose, ConvexPolygon2DReadOnly foothold)
   {
      int i = stepsVisualized.getIntegerValue();

      footholdData[i].unsnappedFootstepPolygon.set(foothold);
      footholdData[i].unsnappedFootstepPolygon.applyTransform(footPose, false);
      footholdData[i].unsnappedFootstepPose.set(footPose);
   }

   @Override
   public void recordFootPoseIsOnBoundary()
   {
      footholdData[stepsVisualized.getIntegerValue()].footPoseIsOnBoundaryOfWorld.set(true);
   }

   @Override
   public void recordSnapTransform(RigidBodyTransformReadOnly snapTransform, PlanarRegion regionSnappedTo)
   {
      int i = stepsVisualized.getIntegerValue();

      footholdData[i].footWasSnapped.set(true);
      footholdData[i].footSnapTranslation.set(snapTransform.getTranslation());
      footholdData[i].regionsSnapped.set(regionSnappedTo);

      footholdData[i].concaveRegionHull.clear();
      for (int vertex = 0; vertex < Math.min(maximumVertices, regionSnappedTo.getConvexHull().getNumberOfVertices()); vertex++)
         footholdData[i].concaveRegionHull.addVertex(regionSnappedTo.getConcaveHullVertex(vertex));
      footholdData[i].concaveRegionHull.update();
      footholdData[i].concaveRegionHull.applyTransform(regionSnappedTo.getTransformToWorld(), false);

      footholdData[i].concaveRegionPose.set(regionSnappedTo.getTransformToLocal());
   }

   @Override
   public void recordWiggleTransform(RigidBodyTransformReadOnly wiggleTransform)
   {
      int i = stepsVisualized.getIntegerValue();

      footholdData[i].footWasWiggled.set(true);
      footholdData[i].footWiggleTranslation.set(wiggleTransform.getTranslation());
   }

   private static class FootholdData
   {
      private final PlanarRegion regionsSnapped = new PlanarRegion();
      private final YoFrameConvexPolygon2D concaveRegionHull;
      private final YoFramePose3D concaveRegionPose;

      private final YoFramePose3D unsnappedFootstepPose;
      private final YoFrameConvexPolygon2D unsnappedFootstepPolygon;

      private final YoBoolean footWasWiggled;
      private final YoBoolean footWasSnapped;
      private final YoBoolean footPoseIsOnBoundaryOfWorld;

      private final YoVector3D footSnapTranslation;
      private final YoVector3D footWiggleTranslation;

      public FootholdData(String suffix, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
      {
         unsnappedFootstepPose = new YoFramePose3D("unsnappedFootstepPose" + suffix, ReferenceFrame.getWorldFrame(), registry);
         unsnappedFootstepPolygon = new YoFrameConvexPolygon2D("unsnappedFootstepPolygon" + suffix, ReferenceFrame.getWorldFrame(), 4, registry);

         concaveRegionHull = new YoFrameConvexPolygon2D("concaveRegionHull" + suffix, ReferenceFrame.getWorldFrame(), maximumVertices, registry);
         concaveRegionPose = new YoFramePose3D("concaveRegionPose" + suffix, ReferenceFrame.getWorldFrame(), registry);

         footWasWiggled = new YoBoolean("footWasWiggled" + suffix, registry);
         footWasSnapped = new YoBoolean("footWasSnapped" + suffix, registry);
         footPoseIsOnBoundaryOfWorld = new YoBoolean("footPoseIsOnBoundaryOfWorld" + suffix, registry);

         footSnapTranslation = new YoVector3D("footSnapTranslation" + suffix, registry);
         footWiggleTranslation = new YoVector3D("footWiggleTranslation" + suffix, registry);

         YoGraphicPolygon regionGraphic = new YoGraphicPolygon("concaveRegionHull" + suffix, concaveRegionHull, concaveRegionPose, 1.0, YoAppearance.Green());

         AppearanceDefinition footAppearance = YoAppearance.Blue();
         footAppearance.setTransparency(0.5);
         YoGraphicPolygon unsnappedGraphic = new YoGraphicPolygon("unsnappedGraphic" + suffix,
                                                                  unsnappedFootstepPolygon,
                                                                  unsnappedFootstepPose,
                                                                  1.0,
                                                                  footAppearance);

         graphicsListRegistry.registerYoGraphic("Foot Snap Data", regionGraphic);
         graphicsListRegistry.registerYoGraphic("Foot Snap Data", unsnappedGraphic);
      }

      public void reset()
      {
         unsnappedFootstepPose.setToNaN();
         unsnappedFootstepPolygon.clear();

         concaveRegionHull.clear();
         concaveRegionPose.setToNaN();

         footSnapTranslation.setToNaN();
         footWiggleTranslation.setToNaN();

         footWasWiggled.set(false);
         footWasSnapped.set(false);
         footPoseIsOnBoundaryOfWorld.set(false);
      }
   }
}
