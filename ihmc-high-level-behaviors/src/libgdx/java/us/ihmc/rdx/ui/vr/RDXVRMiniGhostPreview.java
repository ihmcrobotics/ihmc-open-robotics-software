package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

import java.util.Set;

public class RDXVRMiniGhostPreview
{
   private static final double GHOST_X_HEADSET_OFFSET = 0.3;
   private static final double GHOST_Y_HEADSET_OFFSET = -0.05;
   private static final double GHOST_Z_HEADSET_OFFSET = -0.0;
   private static final double DEGREE_JOYSTICK_INCREMENT = 1.0;
   private static final double CONTROL_JOYSTICK_THRESHOLD = 0.7;
   private final RDXVRContext vrContext;
   private Pose3D ghostPoseHeadsetOffset;
   private final FullHumanoidRobotModel miniGhostFullRobotModel;
   private ReferenceFrame miniGhostFrame;
   private OneDoFJointBasics[] miniGhostOneDoFJointsExcludingHands;
   private RDXMultiBodyGraphic miniGhostRobotGraphic;
   private boolean graphicsInitialized = false;
   private final float opacity;
   private final Color color;

   public RDXVRMiniGhostPreview(String robotName, RobotDefinition robotDefinition, FullHumanoidRobotModel miniGhostFullRobotModel, RDXVRContext vrContext)
   {
      this(robotName, robotDefinition, miniGhostFullRobotModel, vrContext, null, -1.0f);
   }

   public RDXVRMiniGhostPreview(String robotName, RobotDefinition robotDefinition, FullHumanoidRobotModel miniGhostFullRobotModel, RDXVRContext vrContext, Color color, float opacity)
   {
      this.miniGhostFullRobotModel = miniGhostFullRobotModel;
      this.vrContext  = vrContext;
      this.color = color;
      this.opacity = opacity;

      if (isEnabled())
      {
         miniGhostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(miniGhostFullRobotModel);
         miniGhostRobotGraphic = new RDXMultiBodyGraphic(robotName + " (Mini Preview Ghost)");
         miniGhostRobotGraphic.loadRobotModelAndGraphics(robotDefinition, miniGhostFullRobotModel.getElevator(), 0.05, false);
         miniGhostRobotGraphic.setActive(true);
         miniGhostRobotGraphic.create();

         ReferenceFrame headsetFrame= vrContext.getHeadset().getXForwardZUpHeadsetFrame();
         ghostPoseHeadsetOffset = new Pose3D(headsetFrame.getTransformToRoot());
         ghostPoseHeadsetOffset.getTranslation().addX(GHOST_X_HEADSET_OFFSET);
         ghostPoseHeadsetOffset.getTranslation().addY(GHOST_Y_HEADSET_OFFSET);
         ghostPoseHeadsetOffset.getTranslation().addZ(GHOST_Z_HEADSET_OFFSET);
         ghostPoseHeadsetOffset.getRotation().setYawPitchRoll(3 * Math.PI / 4, 0.0, 0.0);
         miniGhostFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(headsetFrame, ghostPoseHeadsetOffset);
      }
   }

   public void setJoint(int index, double q)
   {
      if (isEnabled() && miniGhostRobotGraphic.isActive())
      {
         miniGhostOneDoFJointsExcludingHands[index].setQ(q);
      }
   }

   public void updatePose()
   {
      if (!isEnabled() || !miniGhostRobotGraphic.isActive())
         return;

      // Initialize visual tweaks once the RDXRigidBody is actually available
      if (!graphicsInitialized)
      {
         if (color != null)
            miniGhostRobotGraphic.setColor(color);
         if (opacity >= 0.0f)
            miniGhostRobotGraphic.setOpacity(opacity);
         graphicsInitialized = true;
      }

      updateGhostPoseWithJoystick();
      updateGhostPitchBasedOnFeet();
      miniGhostRobotGraphic.update();
   }

   private void updateGhostPoseWithJoystick()
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         float lateralJoystick  = controller.getJoystickActionData().x();
         if (Math.abs(lateralJoystick) > CONTROL_JOYSTICK_THRESHOLD)
            ghostPoseHeadsetOffset.getRotation().appendYawRotation(Math.signum(lateralJoystick) * Math.toRadians(DEGREE_JOYSTICK_INCREMENT));
         miniGhostFrame.update();
      });
   }

   /**
    * Calculates a new root orientation by adding the pitch difference between the initial
    * orientation and the average orientation of the robot's sole frames.
    */
   private void updateGhostPitchBasedOnFeet()
   {
      miniGhostFullRobotModel.getElevator().updateFramesRecursively();

      Quaternion rootJointInitialOrientation = new Quaternion(miniGhostFrame.getTransformToRoot().getRotation());
      // Get the rotations of the sole frames.
      Quaternion rightSoleQuat = new Quaternion(miniGhostFullRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame().getRotation());
      Quaternion leftSoleQuat = new Quaternion(miniGhostFullRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getRotation());

      // Calculate the midpoint orientation of the sole frames
      Quaternion midSoleRotation = new Quaternion();
      midSoleRotation.interpolate(rightSoleQuat, leftSoleQuat, 0.5);

      // Extract the pitch angle from the root and midpoint rotations.
      double rootPitch = miniGhostFullRobotModel.getRootJoint().getJointPose().getPitch();
      double midSolePitch = midSoleRotation.getPitch();
      // Calculate the difference in pitch.
      double pitchDifference = midSolePitch - rootPitch;

      // Create a new rotation that represents only the pitch difference.
      Quaternion pitchAdjustment = new Quaternion();
      pitchAdjustment.setYawPitchRoll(0, -pitchDifference, 0);

      // Apply the pitch adjustment to the initial root orientation.
      Quaternion finalRootJointOrientation = new Quaternion();
      finalRootJointOrientation.multiply(rootJointInitialOrientation, pitchAdjustment);

      miniGhostFullRobotModel.getRootJoint().setJointOrientation(finalRootJointOrientation);
      miniGhostFullRobotModel.getRootJoint().setJointPosition(miniGhostFrame.getTransformToRoot().getTranslation());

      miniGhostFullRobotModel.getElevator().updateFramesRecursively();
   }

   public void setActive(boolean active)
   {
      if (isEnabled())
         miniGhostRobotGraphic.setActive(active);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isEnabled() && miniGhostRobotGraphic.isActive())
         miniGhostRobotGraphic.getRenderables(renderables, pool, sceneLevels);
   }

   public boolean isEnabled()
   {
      return miniGhostFullRobotModel != null;
   }

   public void destroy()
   {
      if (miniGhostFullRobotModel != null)
         miniGhostRobotGraphic.destroy();
   }
}
