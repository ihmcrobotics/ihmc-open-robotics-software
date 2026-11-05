package us.ihmc.behaviors.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.BehaviorTreeSceneObjectStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;

import java.time.Instant;

/**
 * Tracks a door panel via two YOLO persistent detections, one for the panel and one for the opening mechanism.
 */
public class BehaviorTreeSceneDoorPanelExecutor extends BehaviorTreeSceneObjectExecutor
{
   /**
    * Meters. How far the detected lever sits out from the panel face. The lever bar is 5 cm off the panel, but the
    * detection also covers the rose on the face, so its center is closer. At 0.05 the panel was yawed the other way.
    */
   private static final double MECHANISM_STANDOFF_FROM_PANEL_FACE = 0.025;

   private PersistentDetection panelDetection;
   private final RotationMatrix orientation = new RotationMatrix();
   private final Vector3D mechanismToPanel = new Vector3D();
   private final Vector3D robotToMechanism = new Vector3D();
   private final PersistentDetectionMessageTool persistentDetectionMessageTool = new PersistentDetectionMessageTool();

   public BehaviorTreeSceneDoorPanelExecutor(long id, CRDTInfo crdtInfo, ROS2SyncedRobotModel syncedRobot, BehaviorTreeSceneObjectDefinitionMessage definition)
   {
      super(id, crdtInfo, syncedRobot, definition);
   }

   @Override
   public void update()
   {
      PersistentDetection mechanismDetection = getPersistentDetection();
      if (!frozen.getValue() && mechanismDetection != null && panelDetection != null && mechanismDetection.isStable() && panelDetection.isStable())
      {
         Vector3DBasics mechanismPosition = mechanismDetection.getFilteredTransform().getTranslation();

         // Set orientation to Z-up, then yaw so X axis points from mechanism to panel
         mechanismToPanel.set(panelDetection.getFilteredTransform().getTranslation());
         mechanismToPanel.sub(mechanismPosition);
         mechanismToPanel.setZ(0.0);
         double yaw = Math.atan2(mechanismToPanel.getY(), mechanismToPanel.getX());

         // The mechanism sticks out of the panel face toward the robot, but the panel detection is on the face,
         // so the mechanism-to-panel vector points into the door. Rotate it toward the robot's side to lie along the face.
         double horizontalDistance = mechanismToPanel.norm();
         if (horizontalDistance > MECHANISM_STANDOFF_FROM_PANEL_FACE)
         {
            robotToMechanism.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getPosition());
            robotToMechanism.sub(mechanismPosition);
            double robotSide = mechanismToPanel.getX() * robotToMechanism.getY() - mechanismToPanel.getY() * robotToMechanism.getX();
            yaw += Math.signum(robotSide) * Math.asin(MECHANISM_STANDOFF_FROM_PANEL_FACE / horizontalDistance);
         }
         orientation.setToYawOrientation(yaw);

         if ((!(transform.getValueReadOnly().getRotation().geometricallyEquals(orientation, 1e-5)
             && transform.getValueReadOnly().getTranslation().epsilonEquals(mechanismPosition, 1e-5))))
            transform.getValueAndModify().set(orientation, mechanismPosition);
         referenceFrame.update();
         setValid(true);
      }
   }

   @Override
   public void toMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.toMessage(message);

      if (panelDetection != null)
         persistentDetectionMessageTool.toMessage(syncedRobot, Instant.now(), panelDetection, message.getDoorPanelDetection());
   }

   @Override
   public boolean isStable()
   {
      return super.isStable() && panelDetection != null && panelDetection.isStable();
   }

   public PersistentDetection getDoorPanelPersistentDetection()
   {
      return panelDetection;
   }

   public void setDoorPanelPersistentDetection(PersistentDetection panelDetection)
   {
      this.panelDetection = panelDetection;
   }
}
