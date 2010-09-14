package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.RobotController;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

   public class VisualizeFramesController implements RobotController
   {
      private static final long serialVersionUID = 4252284929932022778L;
      
      private final YoVariableRegistry registry = new YoVariableRegistry("TestFramesController");
      
      private final ArrayList<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

      private final DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("TestFramesController");

      public VisualizeFramesController(ArrayList<ReferenceFrame> referenceFrames, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, double coordinateSystemLength)
      {
         for (ReferenceFrame frame : referenceFrames)
         {
            DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(frame, registry, coordinateSystemLength);
            dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
            dynamicGraphicObjectsList.add(dynamicGraphicReferenceFrame);
         }
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      }

      public void doControl()
      {
         updateDynamicGraphicReferenceFrames();
      }
      
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return "testFramesController";
      }

      private void updateDynamicGraphicReferenceFrames()
      {
         for (DynamicGraphicReferenceFrame frame : dynamicGraphicReferenceFrames)
         {
            frame.update();
         }
      }
   }