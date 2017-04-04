package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;

public class VisualizeFramesController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("VisualizeFramesController");

   private final ArrayList<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   private final YoGraphicsList yoGraphicsList = new YoGraphicsList("TestFramesController");

   public VisualizeFramesController(ArrayList<ReferenceFrame> referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry, double coordinateSystemLength)
   {
      for (ReferenceFrame frame : referenceFrames)
      {
         YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(frame, registry, coordinateSystemLength);
         yoGraphicReferenceFrames.add(yoGraphicReferenceFrame);
         yoGraphicsList.add(yoGraphicReferenceFrame);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   public void doControl()
   {
      updateYoGraphicReferenceFrames();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "visualizeFramesController";
   }

   public void initialize()
   {
   }

   public String getDescription()
   {
      return getName();
   }

   private void updateYoGraphicReferenceFrames()
   {
      for (YoGraphicReferenceFrame frame : yoGraphicReferenceFrames)
      {
         frame.update();
      }
   }
}