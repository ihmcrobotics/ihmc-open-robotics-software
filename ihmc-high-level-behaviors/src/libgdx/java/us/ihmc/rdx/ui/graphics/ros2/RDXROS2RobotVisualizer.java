package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepStatusMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public abstract class RDXROS2RobotVisualizer extends RDXROS2MultiBodyGraphic
{
   protected final RDXBaseUI baseUI;
   protected final ROS2PublishSubscribeAPI ros2;
   private final ImBoolean trackRobot = new ImBoolean(false);
   protected final ImBoolean hideChest = new ImBoolean(false);
   private final ImBoolean showHistory = new ImBoolean(false);
   private final Supplier<RDXFocusBasedCamera> cameraForTrackingSupplier;
   private RDXFocusBasedCamera cameraForTracking;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private final String chestName;
   protected final ROS2SyncedRobotModel syncedRobot;
   protected HumanoidReferenceFrames referenceFramesToUseForSensors;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   protected final ImGuiSliderFloat opacitySlider = new ImGuiSliderFloat("Opacity", "%.2f", 1.0f);
   protected boolean isFading = false;
   private final Pose3D lastHistoryPelvisPose = new Pose3D();
   private final Pose3D currentHistoryPelvisPose = new Pose3D();
   private final RDXTrajectoryGraphic pelvisPoseHistoryGraphic = new RDXTrajectoryGraphic(Color.SKY);
   private final ConcurrentLinkedQueue<MinimalFootstep> completedFootstepThreadBarrier = new ConcurrentLinkedQueue<>();
   private final List<MinimalFootstep> footstepHistory = new ArrayList<>();
   private final RDXFootstepPlanGraphic footstepHistoryGraphic;

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI, ROS2PublishSubscribeAPI ros2, ROS2SyncedRobotModel syncedRobot)
   {
      this(baseUI, ros2, syncedRobot, () -> null);
   }

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI,
                                 ROS2PublishSubscribeAPI ros2,
                                 ROS2SyncedRobotModel syncedRobot,
                                 Supplier<RDXFocusBasedCamera> cameraForTrackingSupplier)
   {
      super(syncedRobot.getRobotModel().getSimpleRobotName() + " Robot Visualizer", StateEstimatorAPI.getRobotConfigurationDataTopic(syncedRobot.getRobotModel().getSimpleRobotName()));
      this.baseUI = baseUI;
      this.ros2 = ros2;
      this.syncedRobot = syncedRobot;
      this.cameraForTrackingSupplier = cameraForTrackingSupplier;

      referenceFramesToUseForSensors = syncedRobot.getReferenceFrames();

      syncedRobot.addRobotConfigurationDataReceivedCallback(getFrequency()::ping);
      previousRobotMidFeetUnderPelvis.setToNaN();
      chestName = syncedRobot.getRobotModel().getJointMap().getChestName();
      footstepHistoryGraphic = new RDXFootstepPlanGraphic(syncedRobot.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      footstepHistoryGraphic.setOpacity(0.7f);
      footstepHistoryGraphic.setColor(RobotSide.LEFT, Color.SKY);
      footstepHistoryGraphic.setColor(RobotSide.RIGHT, Color.SKY);
   }

   /** In simulation we want to use the ground truth frames rather than state estimator frames. */
   public void setReferenceFramesToUseForSensors(HumanoidReferenceFrames referenceFramesToUseForSensors)
   {
      this.referenceFramesToUseForSensors = referenceFramesToUseForSensors;
   }

   @Override
   public void create()
   {
      super.create();
      getMultiBodyGraphic().create();
      if (baseUI != null)
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGuiInput);
      cameraForTracking = cameraForTrackingSupplier.get();
      getMultiBodyGraphic().loadRobotModelAndGraphics(syncedRobot.getRobotModel().getRobotDefinition(), syncedRobot.getFullRobotModel().getElevator());

      ros2.subscribeViaVolatileCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, syncedRobot.getRobotModel().getSimpleRobotName()), footstepStatusMessage ->
      {
         if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
            completedFootstepThreadBarrier.add(new MinimalFootstep(footstepStatusMessage));
      });
   }

   @Override
   public void update()
   {
      if (getMultiBodyGraphic().isRobotLoaded())
      {
         super.update();
         getMultiBodyGraphic().update();

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
            getMultiBodyGraphic().getMultiBody().getRigidBodiesToHide().add(chestName);
         }
         else
         {
            getMultiBodyGraphic().getMultiBody().getRigidBodiesToHide().remove(chestName);
         }
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
      super.renderImGuiWidgets();
      getMultiBodyGraphic().renderImGuiWidgets();

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

      renderSensorInteractables();

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

   protected void renderSensorInteractables()
   {
      if (getMultiBodyGraphic().isRobotLoaded() && opacitySlider.render(0.0f, 1.0f))
      {
         getMultiBodyGraphic().setOpacity(opacitySlider.getFloatValue());
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      getMultiBodyGraphic().setActive(isActive());

      super.getRenderables(renderables, pool, sceneLevels);
      getMultiBodyGraphic().getRenderables(renderables, pool, sceneLevels);

      if (showHistory.get())
      {
         pelvisPoseHistoryGraphic.getRenderables(renderables, pool);
         footstepHistoryGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      super.destroy();
      getMultiBodyGraphic().destroy();
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

   public void visualizeSensors(boolean visualize) {}

   public void fadeVisuals(float finalOpacity, float opacityVariation)
   {
      if (finalOpacity != opacitySlider.getFloatValue())
      {
         isFading = true;
         float newOpacity = (opacitySlider.getFloatValue() > finalOpacity) ? Math.max(opacitySlider.getFloatValue() - opacityVariation, finalOpacity) : Math.min(opacitySlider.getFloatValue() + opacityVariation, finalOpacity);
         opacitySlider.setFloatValue(newOpacity);
         getMultiBodyGraphic().setOpacity(newOpacity);
      }
      else
      {
         isFading = false;
      }
   }

   public boolean isFading()
   {
      return isFading;
   }
}
