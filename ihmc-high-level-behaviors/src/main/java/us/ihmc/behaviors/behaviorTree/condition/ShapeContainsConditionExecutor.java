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
            boolean success = shapeState.getNumberOfPointsContained() >= shapeDefinition.getMinPoints()
                           && shapeState.getNumberOfPointsContained() <= shapeDefinition.getMaxPoints();
            if (success)
               state.getLogger().info("Points contained: %d >= %d <= %d".formatted(shapeDefinition.getMinPoints(),
                                                                                   shapeState.getNumberOfPointsContained(),
                                                                                   shapeDefinition.getMaxPoints()));
            else
               state.getLogger().error("Points contained not in range: %d >= %d <= %d".formatted(shapeDefinition.getMinPoints(),
                                                                                                 shapeState.getNumberOfPointsContained(),
                                                                                                 shapeDefinition.getMaxPoints()));
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
