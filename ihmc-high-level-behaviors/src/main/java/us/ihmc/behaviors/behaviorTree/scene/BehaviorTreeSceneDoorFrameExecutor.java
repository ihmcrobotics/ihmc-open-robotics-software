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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.zed.ZEDImageSensor;

import static behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage.*;

public class BehaviorTreeSceneDoorFrameExecutor extends BehaviorTreeSceneObjectExecutor
{
   private final BehaviorTreeSceneExecutor scene;
   private volatile CUDAShapePointCounter pointCounter = null;

   private BehaviorTreeSceneDoorPanelExecutor doorPanel;

   private RobotSide hingeSide;
   private long latchPostPoints = 0;
   private long hingeRecessPoints = 0;
   private byte doorType = DOOR_TYPE_UNKNOWN;
   private float doorOpenAngle = 0.0f;

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

      if (doorPanel == null)
         return;

      if (!frozen.getValue() && pointCounter != null
       && (depthImage = scene.getImageSensor().getImage(ZEDImageSensor.DEPTH_IMAGE_KEY)) != null)
      {
         latchPostPoints = 0;

         Vector3D panelY = new Vector3D(Axis3D.Y);
         doorPanel.getTransformToWorld().getRotation().transform(panelY);
         Vector3D chestX = new Vector3D(Axis3D.X);
         syncedRobot.getReferenceFrames().getChestFrame().getTransformToWorldFrame().getRotation().transform(chestX);
         hingeSide = panelY.dot(chestX) > 0.0 ? RobotSide.RIGHT : RobotSide.LEFT;

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
            capsuleBottom.addZ(-0.3);
            capsuleTop.set(searchLatchPostPoint);
            capsuleTop.addZ(0.3);

            latchPostPoints = Math.max(latchPostPoints, pointCounter.countPointsInCapsule(depthImage, capsuleBottom, capsuleTop, 0.05f));
            if (latchPostPoints > getMinPostPoints())
            {
               doorOpenAngle = (float) -angle; // Invert so it's latch post relative
               RigidBodyTransform frameTransform = transform.getValueAndModify();
               frameTransform.getTranslation().set(hingePoint);
               EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, searchHingeToLatchPost, frameTransform.getRotation());
               referenceFrame.update();
               break;
            }
         }

         hingeRecessPoints = 0;
         Point3D hingeRecessPoint = new Point3D(-0.1, hingeSide.negateIfLeftSide(0.1), 0.0);
         transform.getValueReadOnly().transform(hingeRecessPoint);
         capsuleBottom.set(hingeRecessPoint);
         capsuleBottom.addZ(-0.3);
         capsuleTop.set(hingeRecessPoint);
         capsuleTop.addZ(0.3);
         hingeRecessPoints += pointCounter.countPointsInCapsule(depthImage, capsuleBottom, capsuleTop, 0.05f);

         if (Math.abs(doorOpenAngle) > Math.toRadians(7.0))
            doorType = (hingeSide == RobotSide.LEFT ? doorOpenAngle > 0 : doorOpenAngle < 0) ? DOOR_TYPE_PUSH : DOOR_TYPE_PULL;
         else
            doorType = hingeRecessPoints > getMinRecessPoints() ? DOOR_TYPE_PUSH : DOOR_TYPE_PULL;

         depthImage.release();
      }
   }

   @Override
   public boolean isStable()
   {
      return latchPostPoints > getMinPostPoints(); // TODO: Filter points in capsule for stable check
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      message.getPersistentDetection().setIsStable(isStable());
      message.setLatchPostPoints(latchPostPoints);
      message.setHingeRecessPoints(hingeRecessPoints);
      message.setDoorType(doorType);
      message.setHingeSide(hingeSide == null ? -1 : hingeSide.toByte());
      message.setDoorOpenAngle(doorOpenAngle);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (pointCounter != null)
         pointCounter.close();
   }

   public byte getDoorType()
   {
      return doorType;
   }

   public RobotSide getHingeSide()
   {
      return hingeSide;
   }

   public float getDoorOpenAngle()
   {
      return doorOpenAngle;
   }
}
