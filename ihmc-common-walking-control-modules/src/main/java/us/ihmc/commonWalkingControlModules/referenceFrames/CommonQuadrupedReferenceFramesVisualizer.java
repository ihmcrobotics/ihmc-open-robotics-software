package us.ihmc.commonWalkingControlModules.referenceFrames;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

public class CommonQuadrupedReferenceFramesVisualizer
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final ArrayList<YoGraphicReferenceFrame> referenceFramesVisualizers = new ArrayList<YoGraphicReferenceFrame>();

   public CommonQuadrupedReferenceFramesVisualizer(CommonQuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      String vizName = referenceFrames.getClass().getSimpleName();
      for (RobotQuadrant robotQudarant : RobotQuadrant.values)
      {
         YoGraphicReferenceFrame ankleZUpFrame = new YoGraphicReferenceFrame(referenceFrames.getAnkleZUpFrame(robotQudarant), registry, false, 0.2);
         yoGraphicsListRegistry.registerYoGraphic(vizName, ankleZUpFrame);
         referenceFramesVisualizers.add(ankleZUpFrame);

         YoGraphicReferenceFrame soleFrame = new YoGraphicReferenceFrame(referenceFrames.getSoleFrame(robotQudarant), registry, false, 0.2);
         yoGraphicsListRegistry.registerYoGraphic(vizName, soleFrame);
         referenceFramesVisualizers.add(soleFrame);
      }

      YoGraphicReferenceFrame midFeetFrame = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), registry, false, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, midFeetFrame);
      referenceFramesVisualizers.add(midFeetFrame);

      YoGraphicReferenceFrame centerOfHipsFrame = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFourHipsFrame(), registry, false, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, centerOfHipsFrame);
      referenceFramesVisualizers.add(centerOfHipsFrame);

      YoGraphicReferenceFrame comFrame = new YoGraphicReferenceFrame(referenceFrames.getCenterOfMassFrame(), registry, false, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, comFrame);
      referenceFramesVisualizers.add(comFrame);

      YoGraphicReferenceFrame bodyFrame = new YoGraphicReferenceFrame(referenceFrames.getBodyFrame(), registry, false, 0.2);
      yoGraphicsListRegistry.registerYoGraphic(vizName, bodyFrame);
      referenceFramesVisualizers.add(bodyFrame);


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
