package us.ihmc.humanoidOperatorInterface.polaris;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.commons.FormattingTools;

public class VehicleModelObjectVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsList yoGraphicsList;
   private final double objectFrameScale = 0.2;
   private final double vehicleFrameScale = 1.0;

   public VehicleModelObjectVisualizer(ReferenceFrame vehicleFrame, VehicleModelObjects vehicleModelObjects, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry)
   {
      yoGraphicsList = new YoGraphicsList("vehicleObjects");

      for (VehicleObject vehicleObject : VehicleObject.values())
      {

         FramePose3D framePose = vehicleModelObjects.getFramePose(vehicleFrame, vehicleObject);

         String objectName = FormattingTools.underscoredToCamelCase(vehicleObject.toString(), false);
         ReferenceFrame objectFrame = new PoseReferenceFrame(objectName, framePose);
         objectFrame.update();

         YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(objectFrame, registry, true, objectFrameScale);
         yoGraphicsList.add(yoGraphicReferenceFrame);
      }

      YoGraphicReferenceFrame vehicleFrameViz = new YoGraphicReferenceFrame(vehicleFrame, registry, true, vehicleFrameScale);
      yoGraphicsList.add(vehicleFrameViz);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
      {
         yoGraphic.update();
      }
   }

   public void setVisible(boolean visible)
   {
      yoGraphicsList.setVisible(visible);
   }
}
