package us.ihmc.gdx.ui.graphics.live;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.function.Supplier;

public class GDXROS2RobotVisualizer extends GDXRobotModelGraphic
{
   private final ImBoolean trackRobot = new ImBoolean(false);
   private final Supplier<FocusBasedGDXCamera> cameraForTrackingSupplier;
   private FocusBasedGDXCamera cameraForTracking;
   private final Point3D previousRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D latestRobotMidFeetUnderPelvis = new Point3D();
   private final Point3D robotTranslationDifference = new Point3D();
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this(robotModel, syncedRobot, () -> null);
   }

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, Supplier<FocusBasedGDXCamera> cameraForTrackingSupplier)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.cameraForTrackingSupplier = cameraForTrackingSupplier;
      syncedRobot.addRobotConfigurationDataReceivedCallback(frequencyPlot::onRecievedMessage);

      previousRobotMidFeetUnderPelvis.setToNaN();
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
      loadRobotModelAndGraphics(robotDefinition, syncedRobot.getFullRobotModel().getElevator(), robotModel);
   }

   @Override
   public void update()
   {
      if (isRobotLoaded())
      {
         super.update();

         if (cameraForTracking != null && trackRobot.get())
         {
            syncedRobot.update();
            latestRobotMidFeetUnderPelvis.set(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getPosition());
            if (!previousRobotMidFeetUnderPelvis.containsNaN())
            {
               robotTranslationDifference.sub(latestRobotMidFeetUnderPelvis, previousRobotMidFeetUnderPelvis);
               cameraForTracking.translateCameraFocusPoint(robotTranslationDifference);
            }
            previousRobotMidFeetUnderPelvis.set(latestRobotMidFeetUnderPelvis);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      frequencyPlot.renderImGuiWidgets();
      if (ImGui.checkbox(labels.get("Track robot"), trackRobot))
      {
         if (!trackRobot.get())
            previousRobotMidFeetUnderPelvis.setToNaN();
      }
   }

   public void destroy()
   {
      super.destroy();
   }
}
