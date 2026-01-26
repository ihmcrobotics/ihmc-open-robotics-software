package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.condition.ShapeContainsConditionDefinition.ContainsType;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDASpherePointCounter;
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
   private final CUDASpherePointCounter spherePointCounter = new CUDASpherePointCounter();
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private int pointsInSphereCUDAOutput = -1;

   public ShapeContainsConditionExecutor(ConditionNodeState state, BehaviorTreeSceneExecutor scene)
   {
      this.scene = scene;
      this.state = state;
      shapeDefinition = state.getDefinition().getShapeContains();
      shapeState = state.getShapeContains();
      zedGrabThread.start();
   }

   public void update()
   {
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
         if (pointsInSphereCUDAOutput >= 0)
         {
            shapeState.setNumberOfPointsContained(pointsInSphereCUDAOutput);
            boolean success = shapeState.getNumberOfPointsContained() >= shapeDefinition.getMinPoints();
            if (success)
               state.getLogger().info("Points contained: %d >= %d".formatted(shapeState.getNumberOfPointsContained(), shapeDefinition.getMinPoints()));
            else
               state.getLogger().error("Points contained not in range: %d < %d".formatted(shapeState.getNumberOfPointsContained(),
                                                                                          shapeDefinition.getMinPoints()));
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

      if (depthImage != null)
      {
         synchronized (this)
         {
            pointsInSphereCUDAOutput = spherePointCounter.countPointsInSphere(depthImage, new Point3D32(spherePose), (float) shapeDefinition.getSphereRadius());
         }
         depthImage.release();
      }
   }
}
