package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.Pair;
import org.lwjgl.openvr.InputDigitalActionData;
import perception_msgs.msg.dds.SceneGraphMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sharedControl.*;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiTransparentPanel;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.visualizers.RDXEdgeDefinedShapeGraphic;
import us.ihmc.rdx.visualizers.RDXSplineGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.*;

/**
 * Class to pack a teleoperated referenceFrame and modify it by using some assistance from the robot.
 * This class also generates a visualization of the assistance through:
 * 1. ghost robot motion
 * 2. spline trajectories for end-effectors
 * 3. std deviation colored region of the autonomy used for assistance
 */
public class RDXVRAssistance implements TeleoperationAssistant, ControlStreamer
{
   private final RDXVRAssistanceStatus status;
   private final SideDependentList<RigidBodyTransform> affordanceToCOMHandTransform = new SideDependentList<>(); // fixed offset hand CoM - link after last wrist joint
   private final ROS2PublishSubscribeAPI ros2;
   private final IHMCROS2Input<SceneGraphMessage> sceneGraphSubscription;
   private final ImBoolean enabledReplay;
   private final ImBoolean enabledIKStreaming;
   private final ImBoolean enabled = new ImBoolean(false);
   private final ProMPAssistant proMPAssistant = new ProMPAssistant();
   private final AffordanceAssistant affordanceAssistant = new AffordanceAssistant();
   private String objectName = "";
   private RigidBodyTransform objectTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                               objectTransformToWorld);
   private boolean previewValidated = false;
   private final FullHumanoidRobotModel ghostRobotModel;
   private final RDXMultiBodyGraphic ghostRobotGraphic;
   private final HashMap<String, RDXSplineGraphic> splineGraphics = new HashMap<>();
   private final HashMap<String, RDXEdgeDefinedShapeGraphic> stdDeviationGraphics = new HashMap<>();
   private final HashMap<String, List<Pose3DReadOnly>> bodyPartReplayMotionMap = new HashMap<>();
   private HashMap<String, Pose3DReadOnly> bodyPartInitialAffordancePoseMap;
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private boolean previewSetToActive = false; // once the validated motion is executed and preview disabled, activate ghostRobotGraphic based on this
   private final ArrayList<KinematicsToolboxOutputStatus> kinematicsToolboxOutputStatusList = new ArrayList<>();
   private boolean firstPreview = true;
   private int replayPreviewCounter = 0;
   private int speedSplineAdjustmentFactor = 1;
   private boolean play = false;
   private double joystickValue;
   private int blendingCounter = 0;
   private RDXVRAssistanceMenu menu;
   boolean sentInitialHandConfiguration = false;
   private final ImBoolean sportMode =  new ImBoolean(false);

   public RDXVRAssistance(DRCRobotModel robotModel, ROS2PublishSubscribeAPI ros2, ImBoolean enabledIKStreaming, ImBoolean enabledReplay)
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
      ghostRobotGraphic.setActive(false);
      ghostRobotGraphic.create();

      sceneGraphSubscription = ros2.subscribe(PerceptionAPI.SCENE_GRAPH.getStatusTopic());

      status = new RDXVRAssistanceStatus(AssistancePhase.DISABLED, RDXVRAssistanceMenuMode.OFF);

      for (RobotSide side: RobotSide.values)
      {
         ReferenceFrame afterLastWristJointFrame = ghostRobotModel.getEndEffectorFrame(side, LimbName.ARM);
         ReferenceFrame comFrame = ghostRobotModel.getHand(side).getBodyFixedFrame();
         affordanceToCOMHandTransform.put(side, comFrame.getTransformToDesiredFrame(afterLastWristJointFrame));
      }
   }

   public void createMenuWindow(RDXImGuiWindowAndDockSystem window)

   {
      menu = new RDXVRAssistanceMenu(window, status);
   }

   /**
    * Process the VR input to activate/deactivate shared control
    */
   public void processInput(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
       {
          // Check if left B button is pressed in order to trigger shared control assistance
          InputDigitalActionData bButton = controller.getBButtonActionData();
          // use left joystick values to control affordance in shared control assistance
          double forwardJoystickValue = controller.getJoystickActionData().y();
          // enable if trigger button has been pressed once. if button is pressed again shared control is stopped
          if (bButton.bChanged() && !bButton.bState())
          {
             setEnabled(!enabled.get());
          }
          joystickValue = forwardJoystickValue;
          if (sportMode.get())
             play = true;
          else
             play = joystickValue > 0 || isPreviewGraphicActive();
       });

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
        {
           if (menu != null)
              menu.update(controller.getXForwardZUpControllerFrame());
        });
   }

   /**
    * 1. ghost robot graphics
    */
   public void updatePreviewModel(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus)
   {
      ghostRobotModel.getRootJoint().setJointPosition(kinematicsToolboxOutputStatus.getDesiredRootPosition());
      ghostRobotModel.getRootJoint().setJointOrientation(kinematicsToolboxOutputStatus.getDesiredRootOrientation());
      for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
      {
         ghostOneDoFJointsExcludingHands[i].setQ(kinematicsToolboxOutputStatus.getDesiredJointAngles().get(i));
      }
      ghostRobotModel.getElevator().updateFramesRecursively();
   }

   /**
    * 1. ghost robot graphics
    */
   public void replayPreviewModel()
   {
      KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus = getPreviewStatus();
      ghostRobotModel.getRootJoint().setJointPosition(kinematicsToolboxOutputStatus.getDesiredRootPosition());
      ghostRobotModel.getRootJoint().setJointOrientation(kinematicsToolboxOutputStatus.getDesiredRootOrientation());
      for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
      {
         ghostOneDoFJointsExcludingHands[i].setQ(kinematicsToolboxOutputStatus.getDesiredJointAngles().get(i));
      }
      ghostRobotModel.getElevator().updateFramesRecursively();
      replaySplinesPreview();

      replayPreviewCounter++;
      if (replayPreviewCounter >= kinematicsToolboxOutputStatusList.size())
         replayPreviewCounter = 0;
   }

   /**
    * 2. spline trajectories for end-effectors
    */
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
            speedSplineAdjustmentFactor = (int) Math.floor((1.0 * kinematicsToolboxOutputStatusList.size()) / (1.0 * entryPartMotion.getValue().size()));
         }
      }
      else
      {
         for (Map.Entry<String, List<Pose3DReadOnly>> entryPartMotion : bodyPartReplayMotionMap.entrySet())
         {
            // since update() method of kinematics streaming can be faster than processVRInput(), the spline size can be shorter than the status list of the ghost robot
            // we do an approximate speed adjustment consisting in waiting before adding the next point of the spline
            int speedAdjuster = replayPreviewCounter / speedSplineAdjustmentFactor;
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

   /**
    * 3. std deviation colored region of the autonomy used for assistance
    */
   private void enableStdDeviationVisualization(String bodyPart)
   {
      if (stdDeviationGraphics.get(bodyPart) == null)
      {
         Point3D[][] edges = createStdDeviationEdges(proMPAssistant.getPriorMean(bodyPart), proMPAssistant.getPriorStdDeviation(bodyPart));
         stdDeviationGraphics.put(bodyPart, new RDXEdgeDefinedShapeGraphic(edges, Color.GREEN, Color.FOREST, 0.3f));
         var stdDeviationGraphic = stdDeviationGraphics.get(bodyPart);
         stdDeviationGraphic.createMainShape();
         stdDeviationGraphic.update();

         Point3D[] startPoints = stdDeviationGraphic.getStartPoints();
         Point3D[] endPoints = stdDeviationGraphic.getEndPoints();
         // Define an array of indices for each rectangular patch
         // these indices are derived from how the edges are created in createStdDeviationEdges() and are always the same
         int[][] patchStartIndices = {{1, 2, 3, 0}, {0, 3, 4, 7}, {7, 4, 5, 6}};
         int[][] patchEndIndices = {{2, 3, 4, 5}, {0, 1, 6, 7}};
         for (int[] indices : patchStartIndices) {
            stdDeviationGraphic.addRectangularPatch(startPoints[indices[0]], startPoints[indices[1]], startPoints[indices[2]], startPoints[indices[3]]);
         }
         for (int[] indices : patchEndIndices) {
            stdDeviationGraphic.addRectangularPatch(endPoints[indices[0]], endPoints[indices[1]], endPoints[indices[2]], endPoints[indices[3]]);
         }

         stdDeviationGraphic.generateMesh();
      }
   }

   /**
    * 3. std deviation colored region of the autonomy used for assistance
    */
   private Point3D[][] createStdDeviationEdges(Point3D[] mean, Point3D[] stdDeviation)
   {
      Point3D[][] edges = new Point3D[8][mean.length];
      for (int edgeNumber = 0; edgeNumber < 8; edgeNumber++)
      {
         for (int i = 0; i < mean.length; i++)
         {
            double x = mean[i].getX();
            double y = mean[i].getY();
            double z = mean[i].getZ();
            // Each edge is identified by an integer index i in the range [0, 7].
            // create the edges in the correct order so to form a convex polygon
            if (edgeNumber == 0 || edgeNumber == 3 || edgeNumber == 4 || edgeNumber == 7)
               x -= stdDeviation[i].getX();
            else
               x += stdDeviation[i].getX();

            if (edgeNumber == 0 || edgeNumber == 1 || edgeNumber == 2 || edgeNumber == 3)
               y -= stdDeviation[i].getY();
            else
               y += stdDeviation[i].getY();

            if (edgeNumber == 0 || edgeNumber == 1 || edgeNumber == 6 || edgeNumber == 7)
               z -= stdDeviation[i].getZ();
            else
               z += stdDeviation[i].getZ();

            edges[edgeNumber][i] = new Point3D(x, y, z);
         }
      }
      return edges;
   }

   /**
    * Observe pose of a bodyPart in order to recognize which task has to be executed and how
    */
   @Override
   public void processFrameInformation(Pose3DReadOnly observedPose, String bodyPart)
   {
      if (proMPAssistant.startedProcessing() && containsBodyPart(bodyPart) && previewSetToActive)
      {
         enableStdDeviationVisualization(bodyPart);
      }

      if (!objectName.isEmpty())
      {
         proMPAssistant.processFrameAndObjectInformation(observedPose, bodyPart, objectName, objectFrame, this);

         if (previewSetToActive)
         {
            if (containsBodyPart(bodyPart) && proMPAssistant.hasPosition(bodyPart))
            {
               // start storing current frames for replay preview with splines
               if (!bodyPartReplayMotionMap.containsKey(bodyPart))
                  bodyPartReplayMotionMap.put(bodyPart, new ArrayList<>());
               else
                  bodyPartReplayMotionMap.get(bodyPart).add(new Pose3D(observedPose));
            }
         }
      }
   }

   @Override
   public boolean readyToPack()
   {
      return proMPAssistant.readyToPack();
   }

   /**
    * Generate assistance: update pose of a bodyPart
    */
   @Override
   public void framePoseToPack(FramePose3D framePose, String bodyPart)
   {
      if (status.getAssistancePhase() == AssistancePhase.PROMP)
      {
         proMPAssistant.framePoseToPack(framePose, bodyPart, play); // pack frame with proMP assistant
         if (containsBodyPart(bodyPart))
         {
            // -- Preview active but not validated yet
            if (previewSetToActive && !previewValidated)
            {
               ghostRobotGraphic.setActive(true); // show ghost robot of preview
               if (proMPAssistant.isCurrentTaskDone()) // if first motion preview is over and not validated yet
                  firstPreview = false;
               if (enabledIKStreaming.get()) // if streaming to controller has been activated again, it means the user validated the motion
               {
                  ghostRobotGraphic.setActive(false); // stop displaying preview ghost robot
                  splineGraphics.clear(); // stop displaying preview splines
                  stdDeviationGraphics.clear(); // stop displaying stdDeviation region
                  previewValidated = true;
                  proMPAssistant.setStartTrajectories(0);
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
            else // -- If user did not use the preview or preview has been validated
            {
               // exit assistance when the current task is over, reactivate it in VR or UI when you want to use it again
               if (!enabledIKStreaming.get() && !isAffordanceActive()) //prevent jump by first disabling streaming to controller below and then shared control here
                  setEnabled(false);
               if (proMPAssistant.isCurrentTaskDone())
                  enabledIKStreaming.set(false); // stop the ik streaming so that you can reposition according to the robot state to avoid jumps in poses
               if (proMPAssistant.inEndZone())
               {
                  if (affordanceAssistant.hasAffordance(objectName))
                     status.setAssistancePhase(AssistancePhase.BLENDING);
               }
            }
         }
      }
      else if (status.getAssistancePhase() == AssistancePhase.BLENDING)
      {
         if (!affordanceAssistant.isActive())
         {
            affordanceAssistant.loadAffordance(objectName, objectFrame);
            bodyPartInitialAffordancePoseMap = affordanceAssistant.getInitialHandPoseMap();
         }
         if (containsBodyPart(bodyPart))

         {
            //define a function alpha that goes from 0 to 1 smoothly, while getting to 1 before the end of the motion
            double x = (double) (blendingCounter) / (proMPAssistant.AFFORDANCE_BLENDING_SAMPLES);
            double alpha = 1.0 / (1 + 4 * Math.exp(-18 * (x - 0.2))); //sigmoid with [X:0,Y:~0],[X:0.6,Y:~1],[X>1,Y:1]
            if(play)
               blendingCounter++;
            proMPAssistant.framePoseToPack(framePose, bodyPart, play); // pack frame with proMP assistant
            if (alpha <= 0.9999)
            {
               if (alpha >= 0.998)
                  alpha = 1;
               // gradually interpolate last promp frame to first affordance frame
               FixedFrameQuaternionBasics arbitratedFrameOrientation = new FrameQuaternion(framePose.getOrientation());
               // Perform slerp for quaternion blending
               arbitratedFrameOrientation.interpolate(bodyPartInitialAffordancePoseMap.get(bodyPart).getOrientation(), alpha);
               FixedFramePoint3DBasics arbitratedFramePosition = new FramePoint3D(framePose.getPosition());
               arbitratedFramePosition.interpolate(bodyPartInitialAffordancePoseMap.get(bodyPart).getPosition(), alpha);
               framePose.getPosition().set(arbitratedFramePosition);
               framePose.getOrientation().set(arbitratedFrameOrientation);
            }
            else
            {
               framePose.set(bodyPartInitialAffordancePoseMap.get(bodyPart));
               status.setAssistancePhase(AssistancePhase.AFFORDANCE);
            }
         }

      }
      else if (status.getAssistancePhase() == AssistancePhase.AFFORDANCE)
      {
         if(!affordanceAssistant.isAffordanceOver())
         {
            if (containsBodyPart(bodyPart))
            {
               affordanceAssistant.framePoseToPack(framePose, bodyPart, play); // pack frame with affordance assistant
            }
         }
         else
         {
            enabledIKStreaming.set(false);
            setEnabled(false);
         }
      }
   }

   @Override
   public void streamOnNotification() {
      LogTools.info("Notified: ", enabledIKStreaming.get());
      if (!enabledIKStreaming.get())
         enabledIKStreaming.set(true);
   }

   public void checkForHandConfigurationUpdates(HandConfigurationListener listener)
   {
      if (status.getAssistancePhase() != AssistancePhase.PROMP)
         affordanceAssistant.checkForHandConfigurationUpdates(listener);
      else if (status.getAssistancePhase() == AssistancePhase.PROMP && !sentInitialHandConfiguration)
      {
         if (listener != null)
            listener.onNotification();
      }
   }

   public void update()
   {
//      if (detectableSceneObjectsSubscription.getMessageNotification().poll() && !proMPAssistant.startedProcessing())
//      {
//         DetectableSceneNodesMessage detectableSceneNodeMessage = detectableSceneObjectsSubscription.getMessageNotification().read();
//         for (var sceneNodeMessage : detectableSceneNodeMessage.getDetectableSceneNodes())
//         {
//            // TODO. update this once Panel and LeverHandle are unified into single detectable "Door"
//            if (sceneNodeMessage.currently_detected_ && !sceneNodeMessage.name_.toString().contains("Panel") && !sceneNodeMessage.name_.toString().contains("Frame"))
//            {
//               objectName = sceneNodeMessage.getNameAsString();
//               MessageTools.toEuclid(sceneNodeMessage.getTransformToWorld(), objectTransformToWorld);
//               objectFrame.update();
//            }
//         }
//      }
      //update menu
      if (!enabled.get() && !objectName.isEmpty()) // if assistance not enabled and objects in the scene
      { // Press left B button to activate
         if(status.getActiveMenu() != RDXVRAssistanceMenuMode.ACTIVATE)
            menu.resetTimer();
         status.setActiveMenu(RDXVRAssistanceMenuMode.ACTIVATE);
      }
      else if (!enabled.get()) // if assistance is off and no object in the scene
         status.setActiveMenu(RDXVRAssistanceMenuMode.OFF);
      else if (enabled.get() && proMPAssistant.startedProcessing() && !proMPAssistant.readyToPack()) // if assistance is on and promp ready
      { // move joysticks
         if(status.getActiveMenu() != RDXVRAssistanceMenuMode.MOVE)
            menu.resetTimer();
         status.setActiveMenu(RDXVRAssistanceMenuMode.MOVE);
      }
      else if ((!previewSetToActive && proMPAssistant.readyToPack()) || (previewSetToActive && previewValidated)) // if assistance is on and no preview, or preview is validated
      {
         if(status.getActiveMenu() != RDXVRAssistanceMenuMode.DIRECT_WITH_JOYSTICK)
            menu.resetTimer();
         status.setActiveMenu(RDXVRAssistanceMenuMode.DIRECT_WITH_JOYSTICK);
         menu.setJoystickPressed(play);

      }
      else if (previewSetToActive && proMPAssistant.readyToPack() && !previewValidated) // if assistance is on and preview
      {
         if(status.getActiveMenu() != RDXVRAssistanceMenuMode.VALIDATE)
            menu.resetTimer();
         status.setActiveMenu(RDXVRAssistanceMenuMode.VALIDATE);
      }
      else
         status.setActiveMenu(RDXVRAssistanceMenuMode.IDLE);

      if (status.getAssistancePhase() == AssistancePhase.PROMP || status.getAssistancePhase() == AssistancePhase.BLENDING)
      {
         if(proMPAssistant.getNumberOfSamples()>0 && !menu.hasProMPSamples())
         {
            menu.setProMPSamples(proMPAssistant.getNumberOfSamples());
            if(!affordanceAssistant.hasAffordance(objectName))
               menu.setHasAffordance(false);
            else
               menu.setHasAffordance(true);
         }
         menu.setCurrentProMPSample(proMPAssistant.getCurrentSample());
      }
      else if (status.getAssistancePhase() == AssistancePhase.AFFORDANCE)
      {
         if(!menu.hasAffordanceSamples())
         {
            menu.setProMPSamples(-1);
            menu.setAffordanceSamples(affordanceAssistant.getNumberOfSamples());
         }
         menu.setCurrentAffordanceSample(affordanceAssistant.getCurrentSample());
      }
   }

   public void renderWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Assistance"), enabled))
         setEnabled(enabled.get());
      ghostRobotGraphic.renderImGuiWidgets();
      if(ImGui.checkbox(labels.get("Sport Mode"), sportMode))
         proMPAssistant.setSportMode(sportMode.get());
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
   }

   public void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
      {
         this.enabled.set(enabled);
         if (enabled)
         {
            firstPreview = true;
            status.setAssistancePhase(AssistancePhase.PROMP);

            if (enabledReplay.get())
               this.enabled.set(false); // check no concurrency with replay

            if (!enabledIKStreaming.get() && !isPreviewGraphicActive() && !sportMode.get())
               this.enabledIKStreaming.set(true);  // if preview disabled we do not want to start the assistance while we're not streaming to the controller
            else if (isPreviewGraphicActive())
               enabledIKStreaming.set(false); // if preview is enabled we do not want to stream to the controller
            previewSetToActive = isPreviewGraphicActive();
            ghostRobotGraphic.setActive(false); // do not show ghost robot immediately, wait that prediction is available
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
            kinematicsToolboxOutputStatusList.clear();
            ghostRobotGraphic.setActive(previewSetToActive); // set it back to what it was (graphic is disabled when using assistance after validation)
            splineGraphics.clear();
            stdDeviationGraphics.clear();
            status.setAssistancePhase(AssistancePhase.DISABLED);
            blendingCounter = 0;
            affordanceAssistant.reset();
            this.enabledIKStreaming.set(false);
            menu.setProMPSamples(-1);
            menu.setAffordanceSamples(-1);
            sentInitialHandConfiguration = false;
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

   public HashMap<String, RDXEdgeDefinedShapeGraphic> getStdDeviationGraphic()
   {
      return stdDeviationGraphics;
   }

   public boolean containsBodyPart(String bodyPart)
   {
      return ((proMPAssistant.startedProcessing() && proMPAssistant.containsBodyPart(bodyPart)) ||
              (affordanceAssistant.isActive() && affordanceAssistant.containsBodyPart(bodyPart)));
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

   public boolean isAffordanceActive()
   {
      return affordanceAssistant.isActive();
   }

   public boolean isAffordancePhase()
   {
      return affordanceAssistant.hasAffordanceStarted();
   }

   public void saveStatusForPreview(KinematicsToolboxOutputStatus status)
   {
      kinematicsToolboxOutputStatusList.add(new KinematicsToolboxOutputStatus(status));
   }

   public KinematicsToolboxOutputStatus getPreviewStatus()
   {
      return kinematicsToolboxOutputStatusList.get(replayPreviewCounter);
   }

   public boolean isFirstPreview()
   {
      return firstPreview;
   }

   public Pair<RobotSide, HandConfiguration> getHandConfiguration()
   {
      if (!sentInitialHandConfiguration)
      {
         sentInitialHandConfiguration = true;
         var initialHandConfiguration = Pair.of(RobotSide.RIGHT, HandConfiguration.HALF_CLOSE);
         return initialHandConfiguration;
      }
      else
         return affordanceAssistant.getHandConfigurationToSend();
   }

   public boolean isPlaying()
   {
      // avoid last frame when affordance is over and do not stream to controller value before deactivating
      // we're safe when we're either not in affordance phase or in affordance phase and affordance is not over yet
      return (status.getAssistancePhase() != AssistancePhase.AFFORDANCE || (affordanceAssistant.hasAffordanceStarted() && !affordanceAssistant.isAffordanceOver()));
   }

   public RDX3DSituatedImGuiTransparentPanel getMenuPanel()
   {
      return menu.getPanel();
   }
}