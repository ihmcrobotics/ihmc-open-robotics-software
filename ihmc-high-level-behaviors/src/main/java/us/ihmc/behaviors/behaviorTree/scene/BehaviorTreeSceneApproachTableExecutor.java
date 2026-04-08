package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class BehaviorTreeSceneApproachTableExecutor extends BehaviorTreeSceneObjectExecutor
{
   private final BehaviorTreeSceneExecutor scene;
   private volatile CUDAShapePointCounter pointCounter;

   private final SideDependentList<FramePoint3D> capsuleCenter = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<Long> tablePoints = new SideDependentList<>(0L, 0L);
   private final SideDependentList<FramePoint3D> capsuleBottom = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<FramePoint3D> capsuleTop = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());

   public BehaviorTreeSceneApproachTableExecutor(long id,
                                                 CRDTInfo crdtInfo,
                                                 ROS2SyncedRobotModel syncedRobot,
                                                 BehaviorTreeSceneObjectDefinitionMessage definition,
                                                 BehaviorTreeSceneExecutor scene)
   {
      super(id, crdtInfo, syncedRobot, definition);

      this.scene = scene;

      ThreadTools.startAsDaemon(() -> pointCounter = new CUDAShapePointCounter(), "ApproachTablePointCounterInitializer");
   }

   @Override
   public void update()
   {
      if (frozen.getValue() || pointCounter == null)
         return;

      RawImage depthImage = scene.getImageSensor().getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
      if (depthImage == null)
         return;

      try
      {
         for (RobotSide side : RobotSide.values)
         {
            capsuleCenter.get(side).setToZero(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame());
            capsuleCenter.get(side).addX(getSearchStartX());
            capsuleCenter.get(side).addY(side.negateIfLeftSide(0.3));
            capsuleCenter.get(side).setZ(0.8);
            tablePoints.put(side, 0L);
         }

         int minPoints = getMinCapsulePoints();
         double search = getSearchStartX();
         double searchLimit = 2.0;
         while (search < searchLimit && (tablePoints.get(RobotSide.LEFT) < minPoints || tablePoints.get(RobotSide.RIGHT) < minPoints))
         {
            for (RobotSide side : RobotSide.values)
            {
               if (tablePoints.get(side) < minPoints)
               {
                  capsuleBottom.get(side).setIncludingFrame(capsuleCenter.get(side));
                  capsuleBottom.get(side).addZ(-0.3);

                  capsuleTop.get(side).setIncludingFrame(capsuleCenter.get(side));
                  capsuleTop.get(side).addZ(0.3);

                  capsuleBottom.get(side).changeFrame(ReferenceFrame.getWorldFrame());
                  capsuleTop.get(side).changeFrame(ReferenceFrame.getWorldFrame());

                  tablePoints.put(side, pointCounter.countPointsInCapsule(depthImage, capsuleBottom.get(side), capsuleTop.get(side), 0.05f));

                  final double stepAt0Meters = 0.01; // Resolution near start
                  final double stepAt2Meters = 0.04; // Resolution at limit
                  final double distanceFromSearchStart = Math.max(0.0, search - getSearchStartX());
                  final double clampedDistance = Math.min(distanceFromSearchStart, searchLimit);
                  final double searchStep = stepAt0Meters + (stepAt2Meters - stepAt0Meters) * (clampedDistance / searchLimit);
                  search += searchStep;

                  capsuleCenter.get(side).setX(search);
               }
            }
         }

         RotationMatrix rotationMatrix = new RotationMatrix();
         Vector3D leftToRight = new Vector3D();
         leftToRight.sub(capsuleBottom.get(RobotSide.RIGHT), capsuleBottom.get(RobotSide.LEFT));
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(new Vector3D(0.0, 1.0, 0.0), leftToRight, rotationMatrix);

         transform.getValue().getTranslation().interpolate(capsuleBottom.get(RobotSide.LEFT), capsuleBottom.get(RobotSide.RIGHT), 0.5);
         transform.getValue().getTranslation().setZ(syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getTransformToRoot().getTranslation().getZ());
         transform.getValue().getRotation().set(rotationMatrix);
         transform.modify();
         referenceFrame.update();
         setValid(true);
      }
      finally
      {
         depthImage.release();
      }
   }

   @Override
   public boolean isStable()
   {
      return pointCounter != null && tablePoints.get(RobotSide.LEFT) > 0 && tablePoints.get(RobotSide.RIGHT) > 0;
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      message.getPersistentDetection().setIsStable(isStable());
      message.setLeftTablePoints(tablePoints.get(RobotSide.LEFT));
      message.setRightTablePoints(tablePoints.get(RobotSide.RIGHT));
      capsuleCenter.get(RobotSide.LEFT).changeFrame(ReferenceFrame.getWorldFrame());
      capsuleCenter.get(RobotSide.RIGHT).changeFrame(ReferenceFrame.getWorldFrame());
      message.getLeftCapsuleCenter().set(capsuleCenter.get(RobotSide.LEFT));
      message.getRightCapsuleCenter().set(capsuleCenter.get(RobotSide.RIGHT));
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (pointCounter != null)
         pointCounter.close();
   }

   public long getLeftTablePoints()
   {
      return tablePoints.get(RobotSide.LEFT);
   }

   public long getRightTablePoints()
   {
      return tablePoints.get(RobotSide.RIGHT);
   }
}
