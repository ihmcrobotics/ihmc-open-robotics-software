package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.FootstepStatusMessage;
import controller_msgs.RobotConfigurationData;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXROS2RobotVisualizer extends RDXROS2SingleTopicVisualizer<RobotConfigurationData>
{
   private final ROS2Node ros2Node;
   private final RDXMultiBodyGraphic multiBodyGraphic;
   private final ROS2Topic<RobotConfigurationData> topic;
   private final ImBoolean trackRobot = new ImBoolean(false);
   private final ImBoolean hideChest = new ImBoolean(false);
   private final ImBoolean showHistory = new ImBoolean(false);
   private RDXFocusBasedCamera cameraForTracking = null;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private final String chestName;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSliderFloat opacitySlider = new ImGuiSliderFloat("Opacity", "%.2f", 1.0f);
   private final Pose3D lastHistoryPelvisPose = new Pose3D();
   private final Pose3D currentHistoryPelvisPose = new Pose3D();
   private final RDXTrajectoryGraphic pelvisPoseHistoryGraphic = new RDXTrajectoryGraphic(Color.SKY);
   private final ConcurrentLinkedQueue<MinimalFootstep> completedFootstepThreadBarrier = new ConcurrentLinkedQueue<>();
   private final List<MinimalFootstep> footstepHistory = new ArrayList<>();
   private final RDXFootstepPlanGraphic footstepHistoryGraphic;
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
      chestName = syncedRobot.getRobotModel().getJointMap().getChestName();
      footstepHistoryGraphic = new RDXFootstepPlanGraphic(syncedRobot.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      footstepHistoryGraphic.setOpacity(0.7f);
      footstepHistoryGraphic.setColor(RobotSide.LEFT, Color.SKY);
      footstepHistoryGraphic.setColor(RobotSide.RIGHT, Color.SKY);
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
   }

   @Override
   public void create()
   {
      super.create();

      multiBodyGraphic.create();
      multiBodyGraphic.loadRobotModelAndGraphics(syncedRobot.getRobotModel().getRobotDefinition(), syncedRobot.getFullRobotModel().getElevator());

      ros2Node.createSubscriptionSampler(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()), sample ->
      {
         if (sample.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
            completedFootstepThreadBarrier.add(new MinimalFootstep(sample));
      });
   }

   @Override
   public void update()
   {
      if (multiBodyGraphic.isRobotLoaded())
      {
         super.update();
         multiBodyGraphic.update();

         if (cameraForTracking != null && trackRobot.get())
         {
            latestRobotMidFeetUnderPelvis.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getPosition());
            if (!previousRobotMidFeetUnderPelvis.containsNaN())
            {
               robotTranslationDifference.sub(latestRobotMidFeetUnderPelvis, previousRobotMidFeetUnderPelvis);
               cameraForTracking.translateCameraFocusPoint(robotTranslationDifference);
            }
            previousRobotMidFeetUnderPelvis.set(latestRobotMidFeetUnderPelvis);
         }

         if (hideChest.get())
         {
            multiBodyGraphic.getMultiBody().getRigidBodiesToHide().add(chestName);
         }
         else
         {
            multiBodyGraphic.getMultiBody().getRigidBodiesToHide().remove(chestName);
         }

         for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
         {
            interactableFrameModel.setShowing(!hideChest.get());
            interactableFrameModel.update();
         }
      }

      if (!snappedToRobotOnStart && syncedRobot.hasReceivedFirstMessage())
      {
         teleportCameraToRobotPelvis();
         snappedToRobotOnStart = true;
      }

      syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToDesiredFrame(currentHistoryPelvisPose, ReferenceFrame.getWorldFrame());
      if (!EuclidCoreMissingTools.epsilonEquals(lastHistoryPelvisPose, currentHistoryPelvisPose, Math.toRadians(2.0), 0.02))
      {
         // FIXME: This can crash the UI when it has too many points
//         lastHistoryPelvisPose.set(currentHistoryPelvisPose);
//         pelvisPoseHistoryGraphic.update(0.01, currentHistoryPelvisPose);
      }

      // Avoid generating the meshes when we aren't showing them, just because the footstep plan graphic isn't super optimized
      if (showHistory.get())
      {
         boolean added = false;
         while (!completedFootstepThreadBarrier.isEmpty())
         {
            added = true;
            footstepHistory.add(completedFootstepThreadBarrier.poll());
         }

         if (added)
            footstepHistoryGraphic.generateMeshes(footstepHistory);
      }

      footstepHistoryGraphic.update();
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
            previousRobotMidFeetUnderPelvis.setToNaN();
      }
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Hide chest"), hideChest);
      if (multiBodyGraphic.isRobotLoaded() && opacitySlider.render(0.0f, 1.0f))
      {
         multiBodyGraphic.setOpacity(opacitySlider.getFloatValue());

         for (RDXInteractableFrameModel interactableFrameModel : interactableFrameModels)
            interactableFrameModel.getModelInstance().setOpacity(opacitySlider.getFloatValue());
      }

      ImGui.checkbox(labels.get("Show History"), showHistory);
      ImGuiTools.previousWidgetTooltip("(The history is always recording.)");
      ImGui.sameLine();
      if (ImGui.button("Clear"))
      {
         pelvisPoseHistoryGraphic.clear();
         footstepHistory.clear();
         footstepHistoryGraphic.clear();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      multiBodyGraphic.setActive(isActive());

      super.getRenderables(renderables, pool, sceneLevels);
      multiBodyGraphic.getRenderables(renderables, pool, sceneLevels);

      if (showHistory.get())
      {
         pelvisPoseHistoryGraphic.getRenderables(renderables, pool);
         footstepHistoryGraphic.getRenderables(renderables, pool);
      }
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
      footstepHistoryGraphic.destroy();
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

   public ImBoolean getHideChest()
   {
      return hideChest;
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
