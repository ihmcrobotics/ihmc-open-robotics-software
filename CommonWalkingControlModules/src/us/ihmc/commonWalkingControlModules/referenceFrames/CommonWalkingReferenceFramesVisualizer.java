package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

public class CommonWalkingReferenceFramesVisualizer implements Updatable, RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ArrayList<DynamicGraphicReferenceFrame> referenceFramesVisualizers = new ArrayList<DynamicGraphicReferenceFrame>();

   public CommonWalkingReferenceFramesVisualizer(CommonWalkingReferenceFrames referenceFrames,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      String vizName = referenceFrames.getClass().getSimpleName();
      for (RobotSide robotSide : RobotSide.values)
      {
         DynamicGraphicReferenceFrame dynamicGraphicObject = new DynamicGraphicReferenceFrame(referenceFrames.getAnkleZUpFrame(robotSide), registry, 0.2);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(vizName, dynamicGraphicObject);
         referenceFramesVisualizers.add(dynamicGraphicObject);
      }

      DynamicGraphicReferenceFrame midFeetFrame = new DynamicGraphicReferenceFrame(referenceFrames.getMidFeetZUpFrame(), registry, 0.2);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(vizName, midFeetFrame);
      referenceFramesVisualizers.add(midFeetFrame);
      DynamicGraphicReferenceFrame comFrame = new DynamicGraphicReferenceFrame(referenceFrames.getCenterOfMassFrame(), registry, 0.2);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(vizName, comFrame);
      referenceFramesVisualizers.add(comFrame);

   }

   @Override
   public void initialize()
   {
      doControl();
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < referenceFramesVisualizers.size(); i++)
      {
         referenceFramesVisualizers.get(i).update();
      }
   }

   @Override
   public void update(double time)
   {
      doControl();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
