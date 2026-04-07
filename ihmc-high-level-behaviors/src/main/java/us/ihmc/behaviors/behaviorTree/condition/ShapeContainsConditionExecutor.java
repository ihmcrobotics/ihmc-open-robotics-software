package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.condition.ShapeContainsConditionDefinition.ContainsType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounterWithColor;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class ShapeContainsConditionExecutor
{
   private final BehaviorTreeSceneExecutor scene;
   private final ConditionNodeState state;
   private final ShapeContainsConditionState shapeState;
   private final ShapeContainsConditionDefinition shapeDefinition;

   private boolean shapeFrameChildOfWorld;
   private ReferenceFrame frame;
   private final FramePoint3D spherePose = new FramePoint3D();
   private final FramePoint3D framePose = new FramePoint3D();
   private CUDAShapePointCounterWithColor spherePointCounter;
   private RepeatingTaskThread zedGrabThread;
   private int pointsInSphereCUDAOutput = -1;

   public ShapeContainsConditionExecutor(ConditionNodeState state, BehaviorTreeSceneExecutor scene)
   {
      this.scene = scene;
      this.state = state;
      shapeDefinition = state.getDefinition().getShapeContains();
      shapeState = state.getShapeContains();

      ThreadTools.startAsDaemon(() -> spherePointCounter = new CUDAShapePointCounterWithColor(), "CreateShapePointCounter");
   }

   public void update()
   {
      if (zedGrabThread == null)
      {
         if (spherePointCounter != null)
         {
            zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
            zedGrabThread.start();
         }
         else
         {
            state.setCanExecute(false);
            return;
         }
      }

      if (state.getIsNextForExecution())
         internalUpdate();

      switch (shapeDefinition.getContainsType())
      {
         case CONTAINS_FRAME -> state.setCanExecute(shapeFrameChildOfWorld && frame != null);
         case CONTAINS_POINTS -> state.setCanExecute(shapeFrameChildOfWorld);
      }
   }

   public void triggerExecution()
   {
      if (zedGrabThread == null)
         return;

      pointsInSphereCUDAOutput = -1;
      internalUpdate();
   }

   private void internalUpdate()
   {
      shapeState.setFrameIsContained(false);

      shapeFrameChildOfWorld = shapeState.getShapeFrame().isChildOfWorld();
      if (shapeFrameChildOfWorld)
      {
         spherePose.setFromReferenceFrame(shapeState.getShapeFrame().getReferenceFrame());
         if (shapeDefinition.getContainsType() == ShapeContainsConditionDefinition.ContainsType.CONTAINS_FRAME)
         {
            frame = scene.findFrameByName(shapeDefinition.getFrameName());
            if (frame != null)
            {
               framePose.setFromReferenceFrame(frame);
               shapeState.setFrameIsContained(framePose.distance(spherePose) <= shapeDefinition.getSphereRadius());
            }
         }
         else
         {
            if (pointsInSphereCUDAOutput >= 0)
               shapeState.setNumberOfPointsContained(pointsInSphereCUDAOutput);

            pointsInSphereCUDAOutput = -1;
            zedGrabThread.setScheduled(1);
         }
      }
   }

   public void updateCurrentlyExecuting()
   {
      if (!shapeFrameChildOfWorld)
      {
         state.getLogger().error("Shape frame is not valid.");
         state.setFailed(true);
         state.setIsExecuting(false);
      }

      if (shapeDefinition.getContainsType() == ContainsType.CONTAINS_FRAME)
      {
         if (frame == null)
            state.getLogger().error("Frame '%s' is null.".formatted(shapeDefinition.getFrameName()));
         state.getLogger()
              .info("Frame '%s' is %s.".formatted(shapeDefinition.getFrameName(), shapeState.getFrameIsContained() ? "contained" : "not contained"));
         state.setFailed(!shapeState.getFrameIsContained());
         state.setIsExecuting(false);
      }
      else
      {
         if (BehaviorTreeTools.findRootNode(state).getPreviewModeEnabled())
         {
            state.getLogger().info("Succeeding blindly in preview mode");
            state.setIsExecuting(false);
         }
         else if (pointsInSphereCUDAOutput >= 0)
         {
            shapeState.setNumberOfPointsContained(pointsInSphereCUDAOutput);
            boolean pointsInRange = shapeState.getNumberOfPointsContained() >= shapeDefinition.getMinPoints()
                                && shapeState.getNumberOfPointsContained() <= shapeDefinition.getMaxPoints();
            boolean success = pointsInRange;
            String pointsCheckSummary = "Points %d in [%d, %d] (%s)".formatted(shapeState.getNumberOfPointsContained(),
                                                                                shapeDefinition.getMinPoints(),
                                                                                shapeDefinition.getMaxPoints(),
                                                                                pointsInRange ? "pass" : "fail");
            String colorCheckSummary = "";
            if (shapeDefinition.getCheckColor())
            {
               float hue360 = shapeState.getAverageHue();
               float sat01 = shapeState.getAverageSaturation();
               float val01 = shapeState.getAverageValue();
               float hue255 = hue360 * (255.0f / 360.0f);
               float sat255 = sat01 * 255.0f;
               float val255 = val01 * 255.0f;

               boolean hueInRange;
               if (shapeDefinition.getHueMin() <= shapeDefinition.getHueMax())
                  hueInRange = hue255 >= shapeDefinition.getHueMin() && hue255 <= shapeDefinition.getHueMax();
               else // wrap-around (red crossing)
                  hueInRange = hue255 >= shapeDefinition.getHueMin() || hue255 <= shapeDefinition.getHueMax();

               boolean saturationInRange = sat255 >= shapeDefinition.getSaturationMin() && sat255 <= shapeDefinition.getSaturationMax();
               boolean valueInRange = val255 >= shapeDefinition.getValueMin() && val255 <= shapeDefinition.getValueMax();
               success &= hueInRange;
               success &= saturationInRange;
               success &= valueInRange;

               String hueRangeDescription = shapeDefinition.getHueMin() <= shapeDefinition.getHueMax()
                     ? "[%d, %d]".formatted(shapeDefinition.getHueMin(), shapeDefinition.getHueMax())
                     : "[%d, 255] U [0, %d]".formatted(shapeDefinition.getHueMin(), shapeDefinition.getHueMax());
               colorCheckSummary = ("%nColor checks:%n"
                     + "H  %.0f in %s (%s)%n"
                     + "S %.0f in [%d, %d] (%s)%n"
                     + "V %.0f in [%d, %d] (%s)").formatted(
                     hue255,
                     hueRangeDescription,
                     hueInRange ? "pass" : "fail",
                     sat255,
                     shapeDefinition.getSaturationMin(),
                     shapeDefinition.getSaturationMax(),
                     saturationInRange ? "pass" : "fail",
                     val255,
                     shapeDefinition.getValueMin(),
                     shapeDefinition.getValueMax(),
                     valueInRange ? "pass" : "fail");
            }

            if (success)
               state.getLogger().info(pointsCheckSummary + colorCheckSummary);
            else
               state.getLogger().error(pointsCheckSummary + colorCheckSummary);
            state.setFailed(!success);
            state.setIsExecuting(false);
         }
      }
   }

   private void zedGrabThread() throws InterruptedException
   {
      ImageSensor zedSensor = scene.getImageSensor();
      zedSensor.waitForGrab();

      RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);

      if (depthImage != null && colorImage != null)
      {
         synchronized (this)
         {
            pointsInSphereCUDAOutput = (int) spherePointCounter.countPointsInSphere(depthImage, colorImage, spherePose, (float) shapeDefinition.getSphereRadius());
            state.getShapeContains().setAverageHue(spherePointCounter.getAverageHue());
            state.getShapeContains().setAverageSaturation(spherePointCounter.getAverageSaturation());
            state.getShapeContains().setAverageValue(spherePointCounter.getAverageValue());
         }
         depthImage.release();
         colorImage.release();
      }
   }
}
