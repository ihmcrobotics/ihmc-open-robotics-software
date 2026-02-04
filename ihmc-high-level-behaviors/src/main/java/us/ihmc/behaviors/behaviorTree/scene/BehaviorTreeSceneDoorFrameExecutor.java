package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
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
   private static final double SEARCH_ANGLE = Math.PI;
   private static final double SEARCH_ANGLE_INCREMENT = SEARCH_ANGLE / 36.0;
   private static final float SEARCH_CAPSULE_RADIUS = 0.08f;

   private final BehaviorTreeSceneExecutor scene;
   private volatile CUDAShapePointCounter pointCounter = null;

   private BehaviorTreeSceneDoorPanelExecutor doorPanel;

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
      super.update();

      RawImage depthImage;

      // Update the frame pose if all conditions are met
      if (pointCounter != null      // Point counter must be initialized
          && doorPanel != null      // A door panel object must be in the scene
          && doorPanel.isStable()   // And its detection must be stable. Finally, we need a depth image from the image sensor
          && (depthImage = scene.getImageSensor().getImage(ZEDImageSensor.DEPTH_IMAGE_KEY)) != null)
      {
         // This object's transform to world will be the hinge post's pose
         RigidBodyTransform transformToWorld = transform.getValueAndModify();
         transformToWorld.set(doorPanel.getTransformToWorld());
         transformToWorld.appendTranslation(DOOR_PANEL_WIDTH + 0.04, 0.0, 0.0);
         transformToWorld.appendYawRotation(Math.PI);

         // Variables used for the search for the latch post
         Pose3D latchPostSearchPose = new Pose3D();
         Point3D frameMiddle = new Point3D();
         Point3D frameTop = new Point3D();

         long maxPoints = 0;
         double yawToAppend = 0.0;

         // Search by looking on the other side of the door panel, then fan out in both directions until we find a post
         for (double angle = 0.0; angle < 0.5 * SEARCH_ANGLE; angle = angle > 0.0 ? -angle : -angle + SEARCH_ANGLE_INCREMENT)
         {
            latchPostSearchPose.set(transformToWorld);
            latchPostSearchPose.appendYawRotation(angle);
            latchPostSearchPose.appendTranslation(DOOR_PANEL_WIDTH + 0.2, 0.0, 0.0);

            frameMiddle.set(latchPostSearchPose.getPosition());
            frameTop.set(latchPostSearchPose.getPosition());
            frameTop.addZ(0.5 * DOOR_PANEL_HEIGHT);

            long points = pointCounter.countPointsInCapsule(depthImage, frameMiddle, frameTop, SEARCH_CAPSULE_RADIUS);
            if (points > maxPoints)
            {
               maxPoints = points;
               yawToAppend = angle;
            }
         }

         transformToWorld.appendYawRotation(yawToAppend);

         depthImage.release();
      }
   }

   public void setDoorPanel(BehaviorTreeSceneDoorPanelExecutor doorPanel)
   {
      this.doorPanel = doorPanel;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (pointCounter != null)
         pointCounter.close();
   }
}
