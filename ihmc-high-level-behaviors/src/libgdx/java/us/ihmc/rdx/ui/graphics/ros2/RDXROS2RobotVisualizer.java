package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiSliderFloat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;
import us.ihmc.rdx.ui.interactable.RDXInteractableOuster;
import us.ihmc.rdx.ui.interactable.RDXInteractableRealsenseD455;
import us.ihmc.rdx.ui.interactable.RDXInteractableZED2i;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.Supplier;

public class RDXROS2RobotVisualizer extends RDXMultiBodyGraphic
{
   private final RDXBaseUI baseUI;
   private final ROS2PublishSubscribeAPI ros2;
   private final ImBoolean trackRobot = new ImBoolean(false);
   private final ImBoolean hideChest = new ImBoolean(false);
   private final Supplier<RDXFocusBasedCamera> cameraForTrackingSupplier;
   private RDXFocusBasedCamera cameraForTracking;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private final DRCRobotModel robotModel;
   private final String chestName;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiSliderFloat opacitySlider = new ImGuiSliderFloat("Opacity", "%.2f", 1.0f);
   private RDXInteractableOuster interactableOuster;
   private RDXInteractableRealsenseD455 interactableRealsenseD455;
   private SideDependentList<RDXInteractableBlackflyFujinon> interactableBlackflyFujinons = new SideDependentList<>();
   private RDXInteractableZED2i interactableZED2i;
   private boolean isFading = false;

   public RDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2PublishSubscribeAPI ros2, ROS2SyncedRobotModel syncedRobot)
   {
      this(null, ros2, robotModel, syncedRobot, () -> null);
   }

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI, ROS2PublishSubscribeAPI ros2, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this(baseUI, ros2, robotModel, syncedRobot, () -> null);
   }

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI,
                                 ROS2PublishSubscribeAPI ros2,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot,
                                 Supplier<RDXFocusBasedCamera> cameraForTrackingSupplier)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.baseUI = baseUI;
      this.ros2 = ros2;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.cameraForTrackingSupplier = cameraForTrackingSupplier;
      syncedRobot.addRobotConfigurationDataReceivedCallback(frequencyPlot::recordEvent);
      previousRobotMidFeetUnderPelvis.setToNaN();
      chestName = robotModel.getJointMap().getChestName();
   }

   @Override
   public void create()
   {
      super.create();
      if (baseUI != null)
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGuiInput);
      cameraForTracking = cameraForTrackingSupplier.get();
      loadRobotModelAndGraphics(robotModel.getRobotDefinition(), syncedRobot.getFullRobotModel().getElevator());

      interactableOuster = new RDXInteractableOuster(baseUI.getPrimary3DPanel(),
                                                     syncedRobot.getReferenceFrames().getOusterLidarFrame(),
                                                     robotModel.getSensorInformation().getOusterLidarTransform());
      interactableOuster.getInteractableFrameModel()
                        .addRemoteTuning(ros2,
                                         PerceptionAPI.OUSTER_TO_CHEST_TUNING,
                                         robotModel.getSensorInformation().getOusterLidarTransform());
      interactableRealsenseD455 = new RDXInteractableRealsenseD455(baseUI.getPrimary3DPanel(),
                                                                   syncedRobot.getReferenceFrames().getSteppingCameraFrame(),
                                                                   robotModel.getSensorInformation().getSteppingCameraTransform());
      interactableRealsenseD455.getInteractableFrameModel()
                               .addRemoteTuning(ros2,
                                                PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                robotModel.getSensorInformation().getSteppingCameraTransform());
      RDXInteractableBlackflyFujinon interactableBlackflyLeftFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel(),
                                                                                                          syncedRobot.getReferenceFrames().getSituationalAwarenessCameraFrame(RobotSide.LEFT),
                                                                                                          robotModel.getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.LEFT));
      interactableBlackflyLeftFujinon.getInteractableFrameModel()
                                     .addRemoteTuning(ros2,
                                                      PerceptionAPI.SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.get(RobotSide.LEFT),
                                                      robotModel.getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.LEFT));
      interactableBlackflyFujinons.set(RobotSide.LEFT, interactableBlackflyLeftFujinon);

      RDXInteractableBlackflyFujinon interactableBlackflyRightFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel(),
                                                                                                           syncedRobot.getReferenceFrames().getSituationalAwarenessCameraFrame(RobotSide.RIGHT),
                                                                                                           robotModel.getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.RIGHT));
      interactableBlackflyRightFujinon.getInteractableFrameModel()
                                      .addRemoteTuning(ros2,
                                                       PerceptionAPI.SITUATIONAL_AWARENESS_CAMERA_TO_PARENT_TUNING.get(RobotSide.RIGHT),
                                                       robotModel.getSensorInformation().getSituationalAwarenessCameraTransform(RobotSide.RIGHT));
      interactableBlackflyFujinons.set(RobotSide.RIGHT, interactableBlackflyRightFujinon);

      interactableZED2i = new RDXInteractableZED2i(baseUI.getPrimary3DPanel(),
                                                   syncedRobot.getReferenceFrames().getExperimentalCameraFrame(),
                                                   robotModel.getSensorInformation().getExperimentalCameraTransform());
      interactableZED2i.getInteractableFrameModel().addRemoteTuning(ros2,
                                                                    PerceptionAPI.EXPERIMENTAL_CAMERA_TO_PARENT_TUNING,
                                                                    robotModel.getSensorInformation().getExperimentalCameraTransform());
   }

   @Override
   public void update()
   {
      if (isRobotLoaded())
      {
         super.update();

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
            getMultiBody().getRigidBodiesToHide().add(chestName);
         }
         else
         {
            getMultiBody().getRigidBodiesToHide().remove(chestName);
         }
         interactableOuster.getInteractableFrameModel().setShowing(!hideChest.get());
         interactableRealsenseD455.getInteractableFrameModel().setShowing(!hideChest.get());
         interactableBlackflyFujinons.forEach((side, blackflyFujinon) -> blackflyFujinon.getInteractableFrameModel().setShowing(!hideChest.get()));
         interactableZED2i.getInteractableFrameModel().setShowing(!hideChest.get());

         interactableOuster.getInteractableFrameModel().update();
         interactableRealsenseD455.getInteractableFrameModel().update();
         interactableBlackflyFujinons.forEach((side, blackflyFujinon) -> blackflyFujinon.getInteractableFrameModel().update());
         interactableZED2i.getInteractableFrameModel().update();
      }
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
      frequencyPlot.renderImGuiWidgets();

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
      if (isRobotLoaded() && opacitySlider.render(0.0f, 1.0f))
      {
         setOpacity(opacitySlider.getFloatValue());
         interactableOuster.getInteractableFrameModel().getModelInstance().setOpacity(opacitySlider.getFloatValue());
         interactableRealsenseD455.getInteractableFrameModel().getModelInstance().setOpacity(opacitySlider.getFloatValue());
         interactableBlackflyFujinons.forEach((side, blackflyFujinon) -> blackflyFujinon.getInteractableFrameModel().getModelInstance().setOpacity(opacitySlider.getFloatValue()));
         interactableZED2i.getInteractableFrameModel().getModelInstance().setOpacity(opacitySlider.getFloatValue());
      }
   }

   public void destroy()
   {
      super.destroy();
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

   public void visualizeSensors(boolean visualize)
   {
      interactableOuster.getInteractableFrameModel().setShowing(visualize);
      interactableRealsenseD455.getInteractableFrameModel().setShowing(visualize);
      interactableBlackflyFujinons.forEach((side, blackflyFujinon) -> blackflyFujinon.getInteractableFrameModel().setShowing(visualize));
      interactableZED2i.getInteractableFrameModel().setShowing(visualize);
   }

   public void fadeVisuals(float finalOpacity, float opacityVariation)
   {
      if (finalOpacity != opacitySlider.getFloatValue())
      {
         isFading = true;
         float newOpacity = (opacitySlider.getFloatValue() > finalOpacity) ? Math.max(opacitySlider.getFloatValue() - opacityVariation, finalOpacity) : Math.min(opacitySlider.getFloatValue() + opacityVariation, finalOpacity);
         opacitySlider.setFloatValue(newOpacity);
         setOpacity(newOpacity);
         interactableOuster.getInteractableFrameModel().getModelInstance().setOpacity(newOpacity);
         interactableRealsenseD455.getInteractableFrameModel().getModelInstance().setOpacity(newOpacity);
         interactableBlackflyFujinons.forEach((side, blackflyFujinon) -> blackflyFujinon.getInteractableFrameModel().getModelInstance().setOpacity(newOpacity));
         interactableZED2i.getInteractableFrameModel().getModelInstance().setOpacity(newOpacity);
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
