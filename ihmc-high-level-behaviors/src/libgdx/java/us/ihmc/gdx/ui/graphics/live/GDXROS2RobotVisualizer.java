package us.ihmc.gdx.ui.graphics.live;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXMultiBodyGraphic;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.function.Supplier;

public class GDXROS2RobotVisualizer extends GDXMultiBodyGraphic
{
   private final ImBoolean trackRobot = new ImBoolean(false);
   private final ImBoolean hideChest = new ImBoolean(false);
   private final Supplier<GDXFocusBasedCamera> cameraForTrackingSupplier;
   private GDXFocusBasedCamera cameraForTracking;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private String chestName;

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this(robotModel, syncedRobot, () -> null);
   }

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, Supplier<GDXFocusBasedCamera> cameraForTrackingSupplier)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
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
      cameraForTracking = cameraForTrackingSupplier.get();
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.Black());
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = robotModel.getJointMap().getHandName(robotSide);
         RobotDefinition.forEachRigidBodyDefinition(robotDefinition.getRigidBodyDefinition(handName),
                                                    body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      }
      loadRobotModelAndGraphics(robotDefinition, syncedRobot.getFullRobotModel().getElevator());
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

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      frequencyPlot.renderImGuiWidgets();
      if (ImGui.button(labels.get("Find My Robot")))
      {
         teleportCameraToRobotPelvis();
      }
      ImGuiTools.previousWidgetTooltip("Brings camera focus to robot's pelvis");
      ImGui.sameLine();
      if (ImGui.checkbox(labels.get("Track robot"), trackRobot))
      {
         if (!trackRobot.get())
            previousRobotMidFeetUnderPelvis.setToNaN();
      }
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Hide chest"), hideChest);
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
      RigidBodyTransform robotPelvisTransform = syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToWorldFrame();
      cameraForTracking.setFocusPointPose(robotPelvisTransform);
   }
}
