package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
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

import java.util.function.Supplier;

public class RDXROS2RobotVisualizer extends RDXMultiBodyGraphic
{
   private final RDXBaseUI baseUI;
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

   public RDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this(null, robotModel, syncedRobot, () -> null);
   }

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this(baseUI, robotModel, syncedRobot, () -> null);
   }

   public RDXROS2RobotVisualizer(RDXBaseUI baseUI,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot,
                                 Supplier<RDXFocusBasedCamera> cameraForTrackingSupplier)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.baseUI = baseUI;
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
}
