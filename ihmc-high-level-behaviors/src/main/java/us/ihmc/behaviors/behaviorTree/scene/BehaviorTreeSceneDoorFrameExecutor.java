package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounter;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class BehaviorTreeSceneDoorFrameExecutor extends BehaviorTreeSceneObjectExecutor
{
   public static final long MINIMUM_POINTS = 300;

   private final BehaviorTreeSceneExecutor scene;
   private volatile CUDAShapePointCounter pointCounter = null;

   private BehaviorTreeSceneDoorPanelExecutor doorPanel;

   private long pointsInCapsule = 0;
   private byte doorType = BehaviorTreeSceneObjectStateMessage.DOOR_TYPE_UNKNOWN;
   private float doorOpenAngle = Float.NaN;

   public BehaviorTreeSceneDoorFrameExecutor(long id,
                                             CRDTInfo crdtInfo,
                                             ROS2SyncedRobotModel syncedRobot,
                                             BehaviorTreeSceneObjectDefinitionMessage definition,
                                             BehaviorTreeSceneExecutor scene)
   {
      super(id, crdtInfo, syncedRobot, definition);

      this.scene = scene;

      ThreadTools.startAsDaemon(() -> pointCounter = new CUDAShapePointCounter(), "PointCounterInitializer");
   }

   public void setDoorPanel(BehaviorTreeSceneDoorPanelExecutor doorPanel)
   {
      this.doorPanel = doorPanel;
   }

   @Override
   public void update()
   {
      RawImage depthImage;

      if (!frozen.getValue() && pointCounter != null
       && (depthImage = scene.getImageSensor().getImage(ZEDImageSensor.DEPTH_IMAGE_KEY)) != null)
      {
         pointsInCapsule = 0;

         double mechanismToHinge = 0.8; // TODO: Will break with different mechanisms
         Point3D hingePoint = new Point3D(mechanismToHinge, 0.0, 0.0);
         doorPanel.getTransformToWorld().transform(hingePoint);

         Point3D nominalLatchPostPoint = new Point3D(-0.16, 0.0, 0.0);
         doorPanel.getTransformToWorld().transform(nominalLatchPostPoint);

         Vector3D searchHingeToLatchPost = new Vector3D();
         Vector3D searchLatchPostPoint = new Vector3D();
         AxisAngle searchAxisAngle = new AxisAngle();
         Point3D capsuleBottom = new Point3D();
         Point3D capsuleTop = new Point3D();

         // Search by looking on the other side of the door panel, then fan out in both directions until we find a post
         for (double angle = 0.0; angle < Math.PI / 2.0; angle = angle > 0.0 ? -angle : -angle + Math.PI / 2.0 / 32.0)
         {
            searchAxisAngle.set(Axis3D.Z, angle);
            searchHingeToLatchPost.sub(nominalLatchPostPoint, hingePoint);
            searchAxisAngle.transform(searchHingeToLatchPost);
            searchLatchPostPoint.add(hingePoint, searchHingeToLatchPost);

            capsuleBottom.set(searchLatchPostPoint);
            capsuleBottom.addZ(-0.2);
            capsuleTop.set(searchLatchPostPoint);
            capsuleTop.addZ(0.2);

            pointsInCapsule = Math.max(pointsInCapsule, pointCounter.countPointsInCapsule(depthImage, capsuleBottom, capsuleTop, 0.05f));
            if (pointsInCapsule > MINIMUM_POINTS)
            {
               RigidBodyTransform frameTransform = transform.getValueAndModify();
               frameTransform.getTranslation().set(hingePoint);
               EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, searchHingeToLatchPost, frameTransform.getRotation());
               referenceFrame.update();

               if (angle > Math.toRadians(7.0)) // TODO: Will need to flip based on hinge side, this is for left hinge side
                  doorType = BehaviorTreeSceneObjectStateMessage.DOOR_TYPE_PULL;
               else if (angle < -Math.toRadians(7.0))
                  doorType = BehaviorTreeSceneObjectStateMessage.DOOR_TYPE_PUSH;

               doorOpenAngle = (float) angle;

               break;
            }
         }

         depthImage.release();
      }
   }

   @Override
   public boolean isStable()
   {
      return pointsInCapsule > MINIMUM_POINTS; // TODO: Filter points in capsule for stable check
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      message.getPersistentDetection().setIsStable(isStable());
      message.setPointsInCapsule(pointsInCapsule);
      message.setDoorType(doorType);
      message.setDoorOpenAngle(doorOpenAngle);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (pointCounter != null)
         pointCounter.close();
   }
}
