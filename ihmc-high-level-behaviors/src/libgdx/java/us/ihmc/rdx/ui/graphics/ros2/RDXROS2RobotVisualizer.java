package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.RobotConfigurationData;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;

import java.util.ArrayList;
import java.util.Set;

public class RDXROS2RobotVisualizer extends RDXROS2SingleTopicVisualizer<RobotConfigurationData>
{
   private final ROS2Node ros2Node;
   private final RDXMultiBodyGraphic multiBodyGraphic;
   private final ROS2Topic<RobotConfigurationData> topic;
   private final ImBoolean trackRobot = new ImBoolean(false);
   private final ImBoolean lockOrientation = new ImBoolean(false);
   private RDXFocusBasedCamera cameraForTracking = null;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private double previousRobotYaw = Double.NaN;
   private double filteredRobotYaw = Double.NaN;
   /** Time constant of the camera yaw filter, in seconds. */
   private static final double YAW_SMOOTHING_TIME_CONSTANT = 0.5;
   private final Vector3D focusPointOffsetFromRobot = new Vector3D();
   private final Point3D newFocusPoint = new Point3D();
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSliderFloat opacitySlider = new ImGuiSliderFloat("Opacity", "%.2f", 1.0f);
   private final ArrayList<RDXInteractableFrameModel> interactableFrameModels = new ArrayList<>();

   private boolean snappedToRobotOnStart = false;

   public RDXROS2RobotVisualizer(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      super(syncedRobot.getRobotModel().getSimpleRobotName() + " Robot Visualizer");

      this.ros2Node = ros2Node;
      this.topic = StateEstimatorAPI.getRobotConfigurationDataTopic(syncedRobot.getRobotModel().getSimpleRobotName());
      this.syncedRobot = syncedRobot;

      multiBodyGraphic = new RDXMultiBodyGraphic(getTitle());
      syncedRobot.addRobotConfigurationDataReceivedCallback(getFrequency()::ping);
      previousRobotMidFeetUnderPelvis.setToNaN();
   }

   public void createAndSetupStandalone(RDXBaseUI baseUI)
   {
      setActive(true);
      setupCameraTracking(baseUI.getPrimary3DPanel().getCamera3D());
      baseUI.getImGuiPanelManager().addPanel(getTitle(), this::renderImGuiWidgets);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGuiInput);
      baseUI.getPrimaryScene().addRenderableProvider(this);
      create();
   }

   public void setupCameraTracking(RDXFocusBasedCamera cameraForTracking)
   {
      this.cameraForTracking = cameraForTracking;
      trackRobot.set(true);
      lockOrientation.set(false);
   }

   @Override
   public void create()
   {
      super.create();

      multiBodyGraphic.create();
      multiBodyGraphic.loadRobotModelAndGraphics(syncedRobot.getRobotModel().getRobotDefinition(), syncedRobot.getFullRobotModel().getElevator());
   }

   @Override
   public void update()
   {
      if (multiBodyGraphic.isRobotLoaded())
      {
         super.update();
         multiBodyGraphic.update();

         if (cameraForTracking != null && trackRobot.get())
            trackRobotWithCamera();

         for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
            interactableFrameModel.update();
      }

      if (!snappedToRobotOnStart && syncedRobot.hasReceivedFirstMessage())
      {
         teleportCameraToRobotPelvis();
         snappedToRobotOnStart = true;
      }
   }

   /**
    * Moves the camera with the robot. With the orientation locked, the camera also turns with the
    * robot's yaw, so it keeps its view of the robot, e.g. always from behind.
    */
   private void trackRobotWithCamera()
   {
      FramePose3DReadOnly robotPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
      latestRobotMidFeetUnderPelvis.set(robotPose.getPosition());
      double latestRobotYaw = filterRobotYaw(robotPose.getYaw());

      if (!previousRobotMidFeetUnderPelvis.containsNaN())
      {
         if (lockOrientation.get() && !Double.isNaN(previousRobotYaw))
         {
            double yawDifference = EuclidCoreTools.trimAngleMinusPiToPi(latestRobotYaw - previousRobotYaw);
            // Turn the focus point about the robot so the whole view stays fixed relative to the robot
            focusPointOffsetFromRobot.sub(cameraForTracking.getFocusPointPose().getPosition(), previousRobotMidFeetUnderPelvis);
            double cos = Math.cos(yawDifference);
            double sin = Math.sin(yawDifference);
            newFocusPoint.set(latestRobotMidFeetUnderPelvis.getX() + cos * focusPointOffsetFromRobot.getX() - sin * focusPointOffsetFromRobot.getY(),
                              latestRobotMidFeetUnderPelvis.getY() + sin * focusPointOffsetFromRobot.getX() + cos * focusPointOffsetFromRobot.getY(),
                              latestRobotMidFeetUnderPelvis.getZ() + focusPointOffsetFromRobot.getZ());
            cameraForTracking.setCameraFocusPoint(newFocusPoint);
            cameraForTracking.rotateCameraYaw(yawDifference);
         }
         else
         {
            robotTranslationDifference.sub(latestRobotMidFeetUnderPelvis, previousRobotMidFeetUnderPelvis);
            cameraForTracking.translateCameraFocusPoint(robotTranslationDifference);
         }
      }
      previousRobotMidFeetUnderPelvis.set(latestRobotMidFeetUnderPelvis);
      previousRobotYaw = latestRobotYaw;
   }

   /**
    * Low-pass filters the robot heading for the camera, so the view does not sway with the pelvis on
    * every step and turns smoothly.
    */
   private double filterRobotYaw(double robotYaw)
   {
      if (Double.isNaN(filteredRobotYaw))
      {
         filteredRobotYaw = robotYaw;
         return filteredRobotYaw;
      }

      double alpha = 1.0 - Math.exp(-Gdx.graphics.getDeltaTime() / YAW_SMOOTHING_TIME_CONSTANT);
      filteredRobotYaw = EuclidCoreTools.trimAngleMinusPiToPi(filteredRobotYaw + alpha * EuclidCoreTools.trimAngleMinusPiToPi(robotYaw - filteredRobotYaw));
      return filteredRobotYaw;
   }

   @Override
   public void processImGuiInput(ImGui3DViewInput input)
   {
      if (input.isWindowHovered() && ImGui.getIO().getKeyCtrl() && ImGui.isKeyReleased('P'))
      {
         teleportCameraToRobotPelvis();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      multiBodyGraphic.renderImGuiWidgets();

      if (ImGui.button(labels.get("Snap to Robot")))
      {
         teleportCameraToRobotPelvis();
      }
      ImGuiTools.previousWidgetTooltip("Moves the camera focus point to the robot's current location.\n (Ctrl + P)");
      ImGui.sameLine();

      if (ImGui.checkbox(labels.get("Track robot"), trackRobot))
      {
         if (!trackRobot.get())
         {
            previousRobotMidFeetUnderPelvis.setToNaN();
            previousRobotYaw = Double.NaN;
            filteredRobotYaw = Double.NaN;
         }
      }
      ImGuiTools.previousWidgetTooltip("Moves the camera with the robot.");
      ImGui.sameLine();
      ImGui.beginDisabled(!trackRobot.get());
      if (ImGui.checkbox(labels.get("Lock orientation"), lockOrientation))
      {
         previousRobotYaw = Double.NaN;
         filteredRobotYaw = Double.NaN;
      }
      ImGui.endDisabled();
      ImGuiTools.previousWidgetTooltip("While tracking, also turns the camera with the robot's heading (yaw only).");
      if (multiBodyGraphic.isRobotLoaded() && opacitySlider.render(0.0f, 1.0f))
      {
         multiBodyGraphic.setOpacity(opacitySlider.getFloatValue());

         for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
            interactableFrameModel.getModelInstance().setOpacity(opacitySlider.getFloatValue());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      multiBodyGraphic.setActive(isActive());

      super.getRenderables(renderables, pool, sceneLevels);
      multiBodyGraphic.getRenderables(renderables, pool, sceneLevels);
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);

      for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
         interactableFrameModel.setShowing(active);
   }

   public void destroy()
   {
      super.destroy();
      multiBodyGraphic.destroy();
   }

   @Override
   public ROS2Topic<RobotConfigurationData> getTopic()
   {
      return topic;
   }

   public RDXMultiBodyGraphic getMultiBodyGraphic()
   {
      return multiBodyGraphic;
   }

   public ImBoolean getTrackRobot()
   {
      return trackRobot;
   }

   public ImBoolean getLockOrientation()
   {
      return lockOrientation;
   }

   public void teleportCameraToRobotPelvis()
   {
      cameraForTracking.setCameraFocusPoint(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getPosition());
   }

   public void setOpacity(float opacity)
   {
      if (opacity != opacitySlider.getFloatValue())
      {
         opacitySlider.setFloatValue(opacity);
         multiBodyGraphic.setOpacity(opacity);

         for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
            interactableFrameModel.getModelInstance().setOpacity(opacity);
      }
   }

   public float getOpacity()
   {
      return opacitySlider.getFloatValue();
   }

   public void attachInteractableFrameModel(RDXInteractableFrameModel interactableFrameModel)
   {
      interactableFrameModels.add(interactableFrameModel);
   }
}
