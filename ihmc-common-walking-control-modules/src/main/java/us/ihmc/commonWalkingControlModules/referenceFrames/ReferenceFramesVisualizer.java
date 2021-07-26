package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ReferenceFramesVisualizer
{
   private static final double DEFAULT_SIZE = 0.2;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final String groupName;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final List<YoGraphicReferenceFrame> referenceFramesVisualizers = new ArrayList<>();

   public ReferenceFramesVisualizer(String groupName, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.groupName = groupName;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      parentRegistry.addChild(registry);
   }

   public void addReferenceFrame(ReferenceFrame referenceFrame)
   {
      addReferenceFrame(referenceFrame, DEFAULT_SIZE);
   }

   public void addReferenceFrame(ReferenceFrame referenceFrame, double size)
   {
      addReferenceFrame(referenceFrame, false, size);
   }

   public void addReferenceFrame(ReferenceFrame referenceFrame, boolean useYawPitchRoll)
   {
      addReferenceFrame(referenceFrame, useYawPitchRoll, DEFAULT_SIZE);
   }

   public void addReferenceFrame(ReferenceFrame referenceFrame, boolean useYawPitchRoll, double size)
   {
      YoGraphicReferenceFrame yoGraphic = new YoGraphicReferenceFrame(referenceFrame, registry, useYawPitchRoll, size);
      yoGraphicsListRegistry.registerYoGraphic(groupName, yoGraphic);
      referenceFramesVisualizers.add(yoGraphic);
   }

   public void update()
   {
      for (int i = 0; i < referenceFramesVisualizers.size(); i++)
      {
         referenceFramesVisualizers.get(i).update();
      }
   }
}
