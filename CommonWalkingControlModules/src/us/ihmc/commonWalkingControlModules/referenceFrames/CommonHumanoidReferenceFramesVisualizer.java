package us.ihmc.commonWalkingControlModules.referenceFrames;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class CommonHumanoidReferenceFramesVisualizer
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ArrayList<YoGraphicReferenceFrame> referenceFramesVisualizers = new ArrayList<YoGraphicReferenceFrame>();

   public CommonHumanoidReferenceFramesVisualizer(CommonHumanoidReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
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
      
      YoGraphicReferenceFrame leftSoleFrmae = new YoGraphicReferenceFrame(referenceFrames.getSoleFrame(RobotSide.LEFT), registry, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, leftSoleFrmae);
      referenceFramesVisualizers.add(leftSoleFrmae);
      
      YoGraphicReferenceFrame rightSoleFrame = new YoGraphicReferenceFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT), registry, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, rightSoleFrame);
      referenceFramesVisualizers.add(rightSoleFrame);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < referenceFramesVisualizers.size(); i++)
      {
         referenceFramesVisualizers.get(i).update();
      }
   }
}
