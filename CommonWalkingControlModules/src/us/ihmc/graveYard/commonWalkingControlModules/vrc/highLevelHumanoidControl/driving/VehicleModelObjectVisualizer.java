package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;


public class VehicleModelObjectVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DynamicGraphicObjectsList dynamicGraphicObjectsList;
   private final double objectFrameScale = 0.2;
   private final double vehicleFrameScale = 1.0;

   public VehicleModelObjectVisualizer(ReferenceFrame vehicleFrame, VehicleModelObjects vehicleModelObjects,
                                DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      dynamicGraphicObjectsList = new DynamicGraphicObjectsList("vehicleObjects");

      for (VehicleObject vehicleObject : VehicleObject.values())
      {

         FramePose framePose = vehicleModelObjects.getFramePose(vehicleFrame, vehicleObject);

         String objectName = FormattingTools.underscoredToCamelCase(vehicleObject.toString(), false);
         ReferenceFrame objectFrame = new PoseReferenceFrame(objectName, framePose);
         objectFrame.update();

         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(objectFrame, registry, objectFrameScale);
         dynamicGraphicObjectsList.add(dynamicGraphicReferenceFrame);
      }

      DynamicGraphicReferenceFrame vehicleFrameViz = new DynamicGraphicReferenceFrame(vehicleFrame, registry, vehicleFrameScale);
      dynamicGraphicObjectsList.add(vehicleFrameViz);

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (DynamicGraphicObject dynamicGraphicObject : dynamicGraphicObjectsList.getDynamicGraphicObjects())
      {
         dynamicGraphicObject.update();
      }
   }

   public void setVisible(boolean visible)
   {
      dynamicGraphicObjectsList.setVisible(visible);
   }
}
