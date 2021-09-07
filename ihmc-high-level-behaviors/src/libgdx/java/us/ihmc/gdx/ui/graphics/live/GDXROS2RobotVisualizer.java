package us.ihmc.gdx.ui.graphics.live;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.graphics.GDXRobotModelGraphic;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

import java.awt.*;
import java.util.List;

public class GDXROS2RobotVisualizer extends GDXRobotModelGraphic
{
//   private boolean trackRobot = false;
//   private FocusBasedGDXCamera cameraForOptionalTracking;
//   private Translate robotTranslate;
//   private Translate savedCameraTranslate;
//   private boolean hasPrepended = false;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("RobotConfigurationData", 1000, 230, 20);
   private volatile long receivedPackets = 0;

   public GDXROS2RobotVisualizer(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      super(robotModel.getSimpleRobotName() + " Robot Visualizer (ROS 2)");
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      syncedRobot.addRobotConfigurationDataReceivedCallback(() -> ++receivedPackets);
   }

   @Override
   public void create()
   {
      super.create();
      RobotDescription robotDescription = robotModel.getRobotDescription();
      overrideModelColors(robotDescription);
      loadRobotModelAndGraphics(robotDescription, syncedRobot.getFullRobotModel().getElevator());
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

   //   public void setTrackRobot(FocusBasedGDXCamera camera, boolean trackRobot)
//   {
//      this.trackRobot = trackRobot;
//      cameraForOptionalTracking = camera;
//      if (!hasPrepended)
//      {
//         hasPrepended = true;
////         robotTranslate = new Translate();
////         cameraForOptionalTracking.prependTransform(robotTranslate);
//      }
//      else if (trackRobot)
//      {
////         cameraForOptionalTracking.getTranslate().setX(savedCameraTranslate.getX());
////         cameraForOptionalTracking.getTranslate().setY(savedCameraTranslate.getY());
////         cameraForOptionalTracking.getTranslate().setZ(savedCameraTranslate.getZ());
//      }
//      else // !trackRobot
//      {
////         savedCameraTranslate = cameraForOptionalTracking.getTranslate().clone();
//      }
//   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      receivedPlot.render(receivedPackets);
   }

   @Override
//   public void update()
//   {
//      if (isRobotLoaded())
//      {
//         super.update();
//
////         if (trackRobot)
////         {
////            FramePose3DReadOnly walkingFrame = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
////
////            robotTranslate.setX(walkingFrame.getPosition().getX());
////            robotTranslate.setY(walkingFrame.getPosition().getY());
////            robotTranslate.setZ(walkingFrame.getPosition().getZ());
////         }
//      }
//   }

   public void destroy()
   {
      super.destroy();
   }
}
