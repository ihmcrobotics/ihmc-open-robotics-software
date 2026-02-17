package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounter;
import us.ihmc.sensors.zed.ZEDImageSensor;

import static us.ihmc.behaviors.simulation.door.DoorModelParameters.DOOR_PANEL_HEIGHT;
import static us.ihmc.behaviors.simulation.door.DoorModelParameters.DOOR_PANEL_WIDTH;

public class BehaviorTreeSceneDoorFrameExecutor extends BehaviorTreeSceneObjectExecutor
{
   public static final double SEARCH_ANGLE = Math.PI;
   public static final double SEARCH_ANGLE_INCREMENT = SEARCH_ANGLE / 36.0;
   public static final float SEARCH_CAPSULE_RADIUS = 0.05f;
   public static final long MINIMUM_POINTS = 1000;

   private final BehaviorTreeSceneExecutor scene;
   private volatile CUDAShapePointCounter pointCounter = null;

   private BehaviorTreeSceneDoorPanelExecutor doorPanel;

   private long pointsInCapsule = 0;
   private double angleToPanel = Double.NaN;
   private boolean latchPostFound = false;

   public BehaviorTreeSceneDoorFrameExecutor(long id,
                                             CRDTInfo crdtInfo,
                                             ROS2SyncedRobotModel syncedRobot,
                                             BehaviorTreeSceneObjectDefinitionMessage definition,
                                             BehaviorTreeSceneExecutor scene)
   {
      super(id, crdtInfo, syncedRobot, definition);

      ThreadTools.startAsDaemon(() -> pointCounter = new CUDAShapePointCounter(), "PointCounterInitializer");

      this.scene = scene;
   }

   @Override
   public void update()
   {
      RawImage depthImage;

      // Update the frame pose if all conditions are met
      if (!frozen.getValue()        // Don't update if frozen
          && pointCounter != null   // Point counter must be initialized
          && doorPanel != null      // A door panel object must be in the scene
          && doorPanel.isStable()   // And its detection must be stable. Finally, we need a depth image from the image sensor
          && (depthImage = scene.getImageSensor().getImage(ZEDImageSensor.DEPTH_IMAGE_KEY)) != null)
      {
         RigidBodyTransform hingePostTransform = new RigidBodyTransform(transform.getValueReadOnly());
         hingePostTransform.set(doorPanel.getTransformToWorld());
         hingePostTransform.appendTranslation(DOOR_PANEL_WIDTH + 0.04, 0.0, 0.0);
         hingePostTransform.appendYawRotation(Math.PI);

         // Variables used for the search for the latch post
         Pose3D latchPostSearchPose = new Pose3D();
         Point3D frameMiddle = new Point3D();
         Point3D frameTop = new Point3D();

         pointsInCapsule = 0;
         latchPostFound = false;

         // Search by looking on the other side of the door panel, then fan out in both directions until we find a post
         for (double angle = 0.0; angle < 0.5 * SEARCH_ANGLE; angle = angle > 0.0 ? -angle : -angle + SEARCH_ANGLE_INCREMENT)
         {
            latchPostSearchPose.set(hingePostTransform);
            latchPostSearchPose.appendYawRotation(angle);
            latchPostSearchPose.appendTranslation(DOOR_PANEL_WIDTH + 0.2, 0.0, 0.0);

            frameMiddle.set(latchPostSearchPose.getPosition());
            frameTop.set(latchPostSearchPose.getPosition());
            frameTop.addZ(0.5 * DOOR_PANEL_HEIGHT);

            long points = pointCounter.countPointsInCapsule(depthImage, frameMiddle, frameTop, SEARCH_CAPSULE_RADIUS);
            if (points > pointsInCapsule)
            {
               pointsInCapsule = points;

               if (points > MINIMUM_POINTS)
               {
                  latchPostFound = true;
                  angleToPanel = angle;
               }
            }
         }

         // Only update this object's transform if we found the latch post
         if (latchPostFound)
         {
            // This object's transform to world will be the hinge post's pose
            // with the x-axis pointing to the latch post
            RigidBodyTransform transformToWorld = transform.getValueAndModify();
            transformToWorld.set(hingePostTransform);
            transformToWorld.appendYawRotation(angleToPanel);
            referenceFrame.update();
         }

         depthImage.release();
      }
   }

   public void setDoorPanel(BehaviorTreeSceneDoorPanelExecutor doorPanel)
   {
      this.doorPanel = doorPanel;
   }

   @Override
   public boolean isStable()
   {
      return doorPanel.isStable() && latchPostFound;
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      message.setPointsInCapsule(pointsInCapsule);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (pointCounter != null)
         pointCounter.close();
   }
}
