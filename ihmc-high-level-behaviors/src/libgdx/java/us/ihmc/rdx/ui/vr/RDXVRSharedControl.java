package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import perception_msgs.msg.dds.DetectedObjectMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sharedControl.ProMPAssistant;
import us.ihmc.behaviors.sharedControl.TeleoperationAssistant;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.perception.ArUcoObjectsPerceptionManager;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.visualizers.RDXSplineGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RDXVRSharedControl implements TeleoperationAssistant
{
   private final ROS2PublishSubscribeAPI ros2;
   private final IHMCROS2Input<DetectedObjectMessage> objectDetectorSubscription;
   private final ImBoolean enabledReplay;
   private final ImBoolean enabledIKStreaming;
   private final ImBoolean enabled = new ImBoolean(false);
   private final ProMPAssistant proMPAssistant = new ProMPAssistant();
   private String objectName = "";
   private RigidBodyTransform objectTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                               objectTransformToWorld);
   private boolean previewValidated = false;
   private final FullHumanoidRobotModel ghostRobotModel;
   private final RDXMultiBodyGraphic ghostRobotGraphic;
   private final HashMap<String, RDXSplineGraphic> splineGraphics = new HashMap<>();
   private final HashMap<String, List<Pose3DReadOnly>> bodyPartReplayMotionMap = new HashMap<>();
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private boolean previewSetToActive = true; // once the validated motion is executed and preview disabled, activate ghostRobotGraphic based on this
   private final ArrayList<KinematicsToolboxOutputStatus> assistanceStatusList = new ArrayList<>();
   private boolean firstPreview = true;
   private int replayPreviewCounter = 0;
   private int speedSplineAdjustmentFactor = 1;

   public RDXVRSharedControl(DRCRobotModel robotModel, ROS2PublishSubscribeAPI ros2, ImBoolean enabledIKStreaming, ImBoolean enabledReplay)
   {
      this.ros2 = ros2;
      this.enabledIKStreaming = enabledIKStreaming;
      this.enabledReplay = enabledReplay;

      // create ghost robot for assistance preview
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("#9370DB").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (Assistance Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      objectDetectorSubscription = ros2.subscribe(ArUcoObjectsPerceptionManager.DETECTED_OBJECT);
   }

   public void processInput(InputDigitalActionData triggerButton)
   {
      // enable if trigger button has been pressed once. if button is pressed again shared control is stopped
      if (triggerButton.bChanged() && !triggerButton.bState())
      {
         setEnabled(!enabled.get());
      }
   }

   public void updatePreviewModel(KinematicsToolboxOutputStatus status)
   {
      ghostRobotModel.getRootJoint().setJointPosition(status.getDesiredRootPosition());
      ghostRobotModel.getRootJoint().setJointOrientation(status.getDesiredRootOrientation());
      for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
      {
         ghostOneDoFJointsExcludingHands[i].setQ(status.getDesiredJointAngles().get(i));
      }
      ghostRobotModel.getElevator().updateFramesRecursively();
   }

   public void replayPreviewModel()
   {
      KinematicsToolboxOutputStatus status = getPreviewStatus();
      ghostRobotModel.getRootJoint().setJointPosition(status.getDesiredRootPosition());
      ghostRobotModel.getRootJoint().setJointOrientation(status.getDesiredRootOrientation());
      for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
      {
         ghostOneDoFJointsExcludingHands[i].setQ(status.getDesiredJointAngles().get(i));
      }
      ghostRobotModel.getElevator().updateFramesRecursively();
      replaySplinesPreview();

      replayPreviewCounter++;
      if (replayPreviewCounter >= assistanceStatusList.size())
         replayPreviewCounter = 0;
   }

   private void replaySplinesPreview()
   {
      // draw spline for the computed trajectories from assistance
      if (replayPreviewCounter == 0)
      {
         for (Map.Entry<String, List<Pose3DReadOnly>> entryPartMotion : bodyPartReplayMotionMap.entrySet())
         {
            if (splineGraphics.containsKey(entryPartMotion.getKey())) // if the spline was previously created, meaning we are at the second replay of full preview
               splineGraphics.get(entryPartMotion.getKey()).clear(); // clear it
            splineGraphics.put(entryPartMotion.getKey(), new RDXSplineGraphic());
            // restart creating the spline from beginning
            splineGraphics.get(entryPartMotion.getKey()).createStart(entryPartMotion.getValue().get(0).getPosition(), Color.BLUE);
            speedSplineAdjustmentFactor = (int) Math.floor((1.0 * assistanceStatusList.size()) / (1.0 * entryPartMotion.getValue().size()));
         }
      }
      else
      {
         for (Map.Entry<String, List<Pose3DReadOnly>> entryPartMotion : bodyPartReplayMotionMap.entrySet())
         {
            // since update() method of kinematics streaming can be faster than processVRInput(), the spline size can be shorter than the status list of the ghost robot
            // we do an approximate speed adjustment consisting in waiting before adding the next point of the spline
            int speedAdjuster = replayPreviewCounter/speedSplineAdjustmentFactor;
            if (speedAdjuster < entryPartMotion.getValue().size() - 1)
            {
               splineGraphics.get(entryPartMotion.getKey()).createAdditionalPoint(entryPartMotion.getValue().get(speedAdjuster).getPosition(), Color.YELLOW);
            }
            else if (speedAdjuster == entryPartMotion.getValue().size() - 1)
            {
               splineGraphics.get(entryPartMotion.getKey()).createEnd(Color.BLUE);
            }

         }
      }
   }

   @Override
   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      if (objectDetectorSubscription.getMessageNotification().poll() && !proMPAssistant.startedProcessing())
      {
         DetectedObjectMessage detectedObjectMessage = objectDetectorSubscription.getMessageNotification().read();
         objectName = detectedObjectMessage.getId().toString();

         MessageTools.toEuclid(detectedObjectMessage.getTransformToWorld(), objectTransformToWorld);
         objectFrame.update();
      }

      if(!objectName.isEmpty())
      {
         proMPAssistant.processFrameAndObjectInformation(observedPose, bodyPart, objectName, objectFrame);

         if (previewSetToActive)
         {
            ghostRobotGraphic.setActive(false); // do not show ghost robot since there is no preview available yet
            // start storing current frames for replay preview with splines
            if (!bodyPartReplayMotionMap.containsKey(bodyPart))
               bodyPartReplayMotionMap.put(bodyPart, new ArrayList<>());
            else
               bodyPartReplayMotionMap.get(bodyPart).add(new Pose3D(observedPose));
         }
      }
   }

   @Override
   public boolean readyToPack()
   {
      return proMPAssistant.readyToPack();
   }

   @Override
   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      proMPAssistant.framePoseToPack(framePose, bodyPart);

      if (previewSetToActive && !previewValidated)  // preview active but not validated yet
      {
         ghostRobotGraphic.setActive(true); // show ghost robot of preview
         if (proMPAssistant.isCurrentTaskDone()) // if first motion preview is over and not validated yet
            firstPreview = false;
         if (enabledIKStreaming.get()) // if streaming to controller has been activated again, it means the user validated the motion
         {
            ghostRobotGraphic.setActive(false); // stop displaying preview ghost robot
            splineGraphics.clear(); // stop displaying preview splines
            previewValidated = true;
         }
         if (!firstPreview) // if second replay or more, keep promp assistant in pause at beginning
            proMPAssistant.setStartTrajectories(0);
         else
         { // if first preview
            // keep storing current frames for replay preview with splines
            if (bodyPartReplayMotionMap.containsKey(bodyPart))
               bodyPartReplayMotionMap.get(bodyPart).add(new Pose3D(framePose));
         }
      }
      else // if user did not use the preview or preview has been validated
      {
         // exit promp assistance when the current task is over, reactivate it in VR or UI when you want to use it again
         if (!enabledIKStreaming.get()) //prevent jump by first disabling streaming to controller and then shared control
            setEnabled(false);
         if (proMPAssistant.isCurrentTaskDone())
            enabledIKStreaming.set(false); // stop the ik streaming so that you can reposition according to the robot state to avoid jumps in poses
      }
   }

   public void renderWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Shared Control"), enabled))
      {
         setEnabled(enabled.get());
      }
      ghostRobotGraphic.renderImGuiWidgets();
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
   }

   private void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
      {
         this.enabled.set(enabled);
         if (enabled)
         {
            firstPreview = true;

            if (enabledReplay.get())
               this.enabled.set(false); // check no concurrency with replay

            if (!enabledIKStreaming.get() && !isPreviewGraphicActive())
               this.enabled.set(false);  // if preview disabled we do not want to start the assistance while we're not streaming to the controller
            else if (isPreviewGraphicActive())
               enabledIKStreaming.set(false); // if preview is enabled we do not want to stream to the controller
            previewSetToActive = isPreviewGraphicActive();
         }
         else // deactivated
         {
            // reset promp assistance
            proMPAssistant.reset();
            objectName = "";
            firstPreview = true;
            previewValidated = false;
            replayPreviewCounter = 0;
            bodyPartReplayMotionMap.clear();
            assistanceStatusList.clear();
            ghostRobotGraphic.setActive(previewSetToActive); // set it back to what it was (graphic is disabled when using assistance after validation)
            splineGraphics.clear();
         }
      }
   }

   public RDXMultiBodyGraphic getGhostPreviewGraphic()
   {
      return ghostRobotGraphic;
   }

   public HashMap<String, RDXSplineGraphic> getSplinePreviewGraphic()
   {
      return splineGraphics;
   }

   public boolean isActive()
   {
      return this.enabled.get();
   }

   public boolean isPreviewActive()
   {
      return previewSetToActive;
   }

   public boolean isPreviewGraphicActive()
   {
      return ghostRobotGraphic.isActive();
   }

   public void saveStatusForPreview(KinematicsToolboxOutputStatus status)
   {
      assistanceStatusList.add(new KinematicsToolboxOutputStatus(status));
   }

   public KinematicsToolboxOutputStatus getPreviewStatus()
   {
      return assistanceStatusList.get(replayPreviewCounter);
   }

   public boolean isFirstPreview()
   {
      return firstPreview;
   }
}
