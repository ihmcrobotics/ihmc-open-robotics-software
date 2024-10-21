package us.ihmc.commonWalkingControlModules.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CommonHumanoidReferenceFramesVisualizer extends ReferenceFramesVisualizer
{
   public CommonHumanoidReferenceFramesVisualizer(CommonHumanoidReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                  YoRegistry parentRegistry, ReferenceFrame... additionalFramesToVisualize)
   {
      super(referenceFrames.getClass().getSimpleName(), yoGraphicsListRegistry, parentRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         addReferenceFrame(referenceFrames.getAnkleZUpFrame(robotSide));
         addReferenceFrame(referenceFrames.getSoleFrame(robotSide));
      }

      addReferenceFrame(referenceFrames.getMidFeetZUpFrame());
      addReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      addReferenceFrame(referenceFrames.getCenterOfMassFrame());
      addReferenceFrame(referenceFrames.getPelvisFrame());

      if (additionalFramesToVisualize != null && additionalFramesToVisualize.length > 0)
      {
         for (ReferenceFrame referenceFrame : additionalFramesToVisualize)
         {
            System.out.println("Visualizing additional frame: " + referenceFrame.getName());
            addReferenceFrame(referenceFrame, true);
         }
      }
   }
}
