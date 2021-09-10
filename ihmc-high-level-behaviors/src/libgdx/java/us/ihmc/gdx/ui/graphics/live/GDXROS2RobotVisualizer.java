package us.ihmc.gdx.ui.graphics.live;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

import java.awt.*;
import java.util.List;
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
   private final ImGuiPlot receivedPlot = new ImGuiPlot("RobotConfigurationData", 1000, 230, 20);
   private volatile long receivedPackets = 0;

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, Supplier<FocusBasedGDXCamera> cameraForTrackingSupplier)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.cameraForTrackingSupplier = cameraForTrackingSupplier;
      syncedRobot.addRobotConfigurationDataReceivedCallback(() -> ++receivedPackets);

      previousRobotMidFeetUnderPelvis.setToNaN();
   }

   @Override
   public void create()
   {
      super.create();
      cameraForTracking = cameraForTrackingSupplier.get();
      RobotDescription robotDescription = robotModel.getRobotDescription();
      overrideModelColors(robotDescription);
      loadRobotModelAndGraphics(robotDescription, syncedRobot.getFullRobotModel().getElevator(), robotModel);
   }

   private void overrideModelColors(RobotDescription robotDescription)
   {
      setModelsToBlack(robotDescription, "l_arm_wry2");
      setModelsToBlack(robotDescription, "l_palm_finger_1_joint");
      setModelsToBlack(robotDescription, "l_finger_1_joint_1");
      setModelsToBlack(robotDescription, "l_finger_1_joint_2");
      setModelsToBlack(robotDescription, "l_finger_1_joint_3");
      setModelsToBlack(robotDescription, "l_palm_finger_2_joint");
      setModelsToBlack(robotDescription, "l_finger_2_joint_1");
      setModelsToBlack(robotDescription, "l_finger_2_joint_2");
      setModelsToBlack(robotDescription, "l_finger_2_joint_3");
      setModelsToBlack(robotDescription, "l_palm_finger_middle_joint");
      setModelsToBlack(robotDescription, "l_finger_middle_joint_1");
      setModelsToBlack(robotDescription, "l_finger_middle_joint_2");
      setModelsToBlack(robotDescription, "l_finger_middle_joint_3");
      setModelsToBlack(robotDescription, "r_arm_wry2");
      setModelsToBlack(robotDescription, "r_palm_finger_1_joint");
      setModelsToBlack(robotDescription, "r_finger_1_joint_1");
      setModelsToBlack(robotDescription, "r_finger_1_joint_2");
      setModelsToBlack(robotDescription, "r_finger_1_joint_3");
      setModelsToBlack(robotDescription, "r_palm_finger_2_joint");
      setModelsToBlack(robotDescription, "r_finger_2_joint_1");
      setModelsToBlack(robotDescription, "r_finger_2_joint_2");
      setModelsToBlack(robotDescription, "r_finger_2_joint_3");
      setModelsToBlack(robotDescription, "r_palm_finger_middle_joint");
      setModelsToBlack(robotDescription, "r_finger_middle_joint_1");
      setModelsToBlack(robotDescription, "r_finger_middle_joint_2");
      setModelsToBlack(robotDescription, "r_finger_middle_joint_3");
   }

   private void setModelsToBlack(RobotDescription robotDescription, String jointName)
   {
      JointDescription handJoint = robotDescription.getJointDescription(jointName);
      if (handJoint != null)
      {
         List<Graphics3DPrimitiveInstruction> instructions = handJoint.getLink().getLinkGraphics().getGraphics3DInstructions();
         for (Graphics3DPrimitiveInstruction instruction : instructions)
         {
            if (instruction instanceof Graphics3DAddModelFileInstruction)
            {
               Graphics3DAddModelFileInstruction addModelFileInstruction = (Graphics3DAddModelFileInstruction) instruction;
               if (addModelFileInstruction.getAppearance() != null)
               {
                  addModelFileInstruction.getAppearance().setTransparency(0.0);
                  addModelFileInstruction.getAppearance().getColor().set(new Color(40, 40, 40));
               }
            }
         }
      }
   }

   @Override
   public void update()
   {
      if (isRobotLoaded())
      {
         super.update();

         if (trackRobot.get())
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
      receivedPlot.render(receivedPackets);
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
