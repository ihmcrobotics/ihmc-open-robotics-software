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

public class PlanarRegionSnapVisualizer
{
   private static final int numberOfStepsToVisualize = 5;
   private static final int maximumVertices = 60;

   private final FootholdData[] footholdData = new FootholdData[numberOfStepsToVisualize];

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger stepsVisualized = new YoInteger("stepsVisualized", registry);

   public PlanarRegionSnapVisualizer(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      for (int i = 0; i < numberOfStepsToVisualize; i++)
         footholdData[i] = new FootholdData("" + i, registry, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      stepsVisualized.set(-1);

//      for (int i = 0; i < numberOfStepsToVisualize; i++)
//         footholdData[i].reset();
   }

   public void setFootIndex(int index)
   {
      stepsVisualized.set(index);
      if (index < footholdData.length)
         footholdData[index].reset();
   }

   public void recordUnadjustedFootstep(FramePose3DReadOnly footPose, ConvexPolygon2DReadOnly foothold)
   {
      int i = stepsVisualized.getIntegerValue();
      if (i >= numberOfStepsToVisualize)
         return;

      footholdData[i].unsnappedFootstepPolygon.set(foothold);
      footholdData[i].unsnappedFootstepPose.set(footPose);
   }

   public void recordFootPoseIsOnBoundary()
   {
      if (stepsVisualized.getIntegerValue() >= numberOfStepsToVisualize)
         return;

      footholdData[stepsVisualized.getIntegerValue()].footPoseIsOnBoundaryOfWorld.set(true);
   }

   public void recordSnapTransform(int regionsUnderFoot, RigidBodyTransformReadOnly snapTransform, PlanarRegion regionSnappedTo)
   {
      int i = stepsVisualized.getIntegerValue();

      if (i >= numberOfStepsToVisualize)
         return;

      footholdData[i].regionsUnderFoot.set(regionsUnderFoot);
      footholdData[i].footWasSnapped.set(true);
      footholdData[i].footSnapTranslation.set(snapTransform.getTranslation());

      if (regionSnappedTo != null)
      {
         footholdData[i].footHadRegion.set(true);
         footholdData[i].concaveRegionHull.clear();
         for (int vertex = 0; vertex < Math.min(maximumVertices, regionSnappedTo.getConcaveHull().size()); vertex++)
            footholdData[i].concaveRegionHull.addVertex(regionSnappedTo.getConcaveHullVertex(vertex));
         footholdData[i].concaveRegionHull.update();

         footholdData[i].concaveRegionPose.set(regionSnappedTo.getTransformToWorld());
         footholdData[i].concaveRegionPose.prependTranslation(0.0, 0.0, 0.001);
      }
      else
      {
         footholdData[i].concaveRegionHull.clear();
         footholdData[i].concaveRegionPose.setToNaN();
      }
   }

   public void recordWiggleTransform(int wiggleIterations, RigidBodyTransformReadOnly wiggleTransform)
   {
      int i = stepsVisualized.getIntegerValue();

      if (i >= numberOfStepsToVisualize)
         return;

      footholdData[i].footWasWiggled.set(true);
      footholdData[i].footWiggleTranslation.set(wiggleTransform.getTranslation());
      footholdData[i].wiggleIterations.set(wiggleIterations);
   }

   private static class FootholdData
   {
      private final YoFrameConvexPolygon2D concaveRegionHull;
      private final YoFramePose3D concaveRegionPose;

      private final YoFramePose3D unsnappedFootstepPose;
      private final YoFrameConvexPolygon2D unsnappedFootstepPolygon;

      private final YoBoolean footWasWiggled;
      private final YoBoolean footWasSnapped;
      private final YoBoolean footHadRegion;
      private final YoBoolean footPoseIsOnBoundaryOfWorld;
      private final YoInteger regionsUnderFoot;
      private final YoInteger wiggleIterations;

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
         footHadRegion = new YoBoolean("footHadRegion" + suffix, registry);
         regionsUnderFoot = new YoInteger("regionsUnderFoot" + suffix, registry);
         footPoseIsOnBoundaryOfWorld = new YoBoolean("footPoseIsOnBoundaryOfWorld" + suffix, registry);

         footSnapTranslation = new YoVector3D("footSnapTranslation" + suffix, registry);
         footWiggleTranslation = new YoVector3D("footWiggleTranslation" + suffix, registry);
         wiggleIterations = new YoInteger("wiggleIterations" + suffix, registry);

         AppearanceDefinition regionAppearance = YoAppearance.Green();
         regionAppearance.setTransparency(0.75);
         YoGraphicPolygon regionGraphic = new YoGraphicPolygon("concave Region Hull " + suffix, concaveRegionHull, concaveRegionPose, 1.0, regionAppearance);

         AppearanceDefinition footAppearance = YoAppearance.Blue();
         footAppearance.setTransparency(0.5);
         YoGraphicPolygon unsnappedGraphic = new YoGraphicPolygon("unsnapped Graphic " + suffix,
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
         footHadRegion.set(false);
         regionsUnderFoot.set(-1);
         wiggleIterations.set(-1);
         footPoseIsOnBoundaryOfWorld.set(false);
      }
   }
}
