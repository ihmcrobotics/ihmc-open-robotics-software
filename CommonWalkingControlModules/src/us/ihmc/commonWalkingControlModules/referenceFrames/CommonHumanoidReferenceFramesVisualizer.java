package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class CommonHumanoidReferenceFramesVisualizer implements Updatable, RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ArrayList<YoGraphicReferenceFrame> referenceFramesVisualizers = new ArrayList<YoGraphicReferenceFrame>();

   public CommonHumanoidReferenceFramesVisualizer(CommonHumanoidReferenceFrames referenceFrames,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String vizName = referenceFrames.getClass().getSimpleName();
      for (RobotSide robotSide : RobotSide.values)
      {
         YoGraphicReferenceFrame yoGraphic = new YoGraphicReferenceFrame(referenceFrames.getAnkleZUpFrame(robotSide), registry, 0.2);
         yoGraphicsListRegistry.registerYoGraphic(vizName, yoGraphic);
         referenceFramesVisualizers.add(yoGraphic);
      }

      YoGraphicReferenceFrame midFeetFrame = new YoGraphicReferenceFrame(referenceFrames.getMidFeetZUpFrame(), registry, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, midFeetFrame);
      referenceFramesVisualizers.add(midFeetFrame);
      YoGraphicReferenceFrame comFrame = new YoGraphicReferenceFrame(referenceFrames.getCenterOfMassFrame(), registry, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, comFrame);
      referenceFramesVisualizers.add(comFrame);
      YoGraphicReferenceFrame pelvisFrame = new YoGraphicReferenceFrame(referenceFrames.getPelvisFrame(), registry, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, pelvisFrame);
      referenceFramesVisualizers.add(pelvisFrame);

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
