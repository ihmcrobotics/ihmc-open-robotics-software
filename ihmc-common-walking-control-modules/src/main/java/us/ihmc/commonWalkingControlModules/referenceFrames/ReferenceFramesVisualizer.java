package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ReferenceFramesVisualizer implements SCS2YoGraphicHolder
{
   private static final double DEFAULT_SIZE = 0.2;

   private final YoRegistry registry;

   private final String groupName;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final List<ReferenceFrameViz> referenceFrameVisualizers = new ArrayList<>();

   public ReferenceFramesVisualizer(String groupName, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this("", groupName, yoGraphicsListRegistry, parentRegistry);
   }

   public ReferenceFramesVisualizer(String namespacePrefix, String groupName, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.groupName = groupName;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      registry = new YoRegistry(namespacePrefix + getClass().getSimpleName());
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
      referenceFrameVisualizers.add(new ReferenceFrameViz(referenceFrame, useYawPitchRoll, size));
   }

   public void update()
   {
      for (int i = 0; i < referenceFrameVisualizers.size(); i++)
      {
         referenceFrameVisualizers.get(i).update();
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < referenceFrameVisualizers.size(); i++)
      {
         group.addChild(referenceFrameVisualizers.get(i).getSCS2YoGraphic());
      }
      return group;
   }

   private class ReferenceFrameViz
   {
      private final ReferenceFrame frame;
      private final YoFramePose3D pose;
      private final YoFramePoseUsingYawPitchRoll poseUsingYawPitchRoll;
      private final double size;

      public ReferenceFrameViz(ReferenceFrame frame, boolean useYawPitchRoll, double size)
      {
         this.frame = frame;
         this.size = size;
         if (useYawPitchRoll)
         {
            pose = null;
            poseUsingYawPitchRoll = new YoFramePoseUsingYawPitchRoll(frame.getName(), ReferenceFrame.getWorldFrame(), registry);
            if (yoGraphicsListRegistry != null)
               yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicCoordinateSystem(frame.getName(), poseUsingYawPitchRoll, size));
         }
         else
         {
            pose = new YoFramePose3D(frame.getName(), ReferenceFrame.getWorldFrame(), registry);
            poseUsingYawPitchRoll = null;
            if (yoGraphicsListRegistry != null)
               yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicCoordinateSystem(frame.getName(), pose, size));
         }
      }

      public void update()
      {
         if (pose != null)
            pose.setFromReferenceFrame(frame);
         else
            poseUsingYawPitchRoll.setFromReferenceFrame(frame);
      }

      public YoGraphicDefinition getSCS2YoGraphic()
      {
         if (pose != null)
            return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(frame.getName(), pose, size);
         else
            return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(frame.getName(), poseUsingYawPitchRoll, size);
      }
   }
}
