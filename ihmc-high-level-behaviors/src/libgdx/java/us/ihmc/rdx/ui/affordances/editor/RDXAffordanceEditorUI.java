package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.*;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class RDXAffordanceEditorUI
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide side = RobotSide.RIGHT;

   private RDXInteractableSakeGripper interactableHand;
   private final RigidBodyTransform handTransformToWorld = new RigidBodyTransform();
   private final FramePose3D handPose = new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformToWorld);
   private RDXInteractableObjectBuilder objectBuilder;
   private final PoseReferenceFrame objectFrame = new PoseReferenceFrame("objectFrame", ReferenceFrame.getWorldFrame());
   private ReferenceFrame initialObjectFrame;
   private ModifiableReferenceFrame affordanceFrame = new ModifiableReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private final float[] gripperClosure = new float[1];
   private static final double DEFAULT_DURATION = 1.0;
   private static final double LINEAR_VELOCITY = 0.1;
   private static final double ANGULAR_VELOCITY = 1.0; // [rad/s] for the sake gripper this is ~= to 0.1 m/s for a point on the edge of the gripper
   // affordance poses
   private RDXAffordanceFrame graspFrame;
   private RDXAffordanceFrames preGraspFrames;
   private RDXAffordanceFrames postGraspFrames;

   public boolean affordancePoseLocked = false;
   private boolean handLocked = false;
   private PoseReferenceFrame handLockedFrame;
   private RDXActiveAffordanceMenu[] activeMenu;
   private boolean playing = false;

   private final ImGuiInputText textInput = new ImGuiInputText("Enter file name to save/load");
   private String fileName = "";
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/affordances");
   private final ArrayList<double[]> csvDataMatrix = new ArrayList<>();

   public RDXAffordanceEditorUI()

   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            objectBuilder = new RDXInteractableObjectBuilder(baseUI, PredefinedSceneNodeLibrary.defaultObjects());
            baseUI.getImGuiPanelManager().addPanel(objectBuilder.getWindowName(), objectBuilder::renderImGuiWidgets);

            handTransformToWorld.getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
            handTransformToWorld.getTranslation().set(-0.5, 0, 0);
            interactableHand = new RDXInteractableSakeGripper(baseUI.getPrimary3DPanel(), handTransformToWorld);

            activeMenu = new RDXActiveAffordanceMenu[1];
            activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
            preGraspFrames = new RDXAffordanceFrames(interactableHand,
                                                     handTransformToWorld,
                                                     handPose,
                                                     objectBuilder.getSelectedObject().getTransformToWorld(),
                                                     activeMenu,
                                                     new ArrayList<>(Arrays.asList(new Color(0xFFE4B5FF),
                                                                                   new Color(0xFF8C00FF),
                                                                                   new Color(0xFFDAB9FF),
                                                                                   new Color(0xFF6600FF),
                                                                                   new Color(0xFFA07AFF))));
            activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
            graspFrame = new RDXAffordanceFrame(interactableHand,
                                                handTransformToWorld,
                                                handPose,
                                                objectBuilder.getSelectedObject().getTransformToWorld(),
                                                activeMenu,
                                                Color.BLACK);
            activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
            postGraspFrames = new RDXAffordanceFrames(interactableHand,
                                                      handTransformToWorld,
                                                      handPose,
                                                      objectBuilder.getSelectedObject().getTransformToWorld(),
                                                      activeMenu,
                                                      new ArrayList<>(Arrays.asList(new Color(0xD8BFD8FF),
                                                                                    new Color(0xBA55D3FF),
                                                                                    new Color(0x9932CCFF),
                                                                                    new Color(0x8A2BE2FF),
                                                                                    new Color(0x4B0082FF))));
            activeMenu[0] = RDXActiveAffordanceMenu.NONE;
            baseUI.getPrimaryScene().addRenderableProvider(interactableHand);
            baseUI.getImGuiPanelManager().addPanel(interactableHand.getPose3DGizmo().createTunerPanel("hand"));
            baseUI.getPrimaryScene().addRenderableProvider(objectBuilder.getSelectedObject());
            baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceEditorUI.this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Affordance Panel", RDXAffordanceEditorUI.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public void update()
   {
      if (objectBuilder.isAnyObjectSelected())
      {
         FramePose3D objectPose = new FramePose3D(ReferenceFrame.getWorldFrame(), objectBuilder.getSelectedObject().getTransformToWorld());
         // when editing post grasp poses, we want to move the object frame and the hand together
         if (activeMenu[0] == RDXActiveAffordanceMenu.POST_GRASP && handLocked)
         {
            // used to update the hand pose according to object pose in post-grasping once fixed contact with object
            if (!affordancePoseLocked)
            {
               objectFrame.setPoseAndUpdate(objectPose);
               handLockedFrame = new PoseReferenceFrame("handFrame", objectFrame);
               handPose.changeFrame(objectFrame);
               handLockedFrame.setPoseAndUpdate(handPose);
               handPose.changeFrame(ReferenceFrame.getWorldFrame());
               affordancePoseLocked = true;
            }
            objectFrame.setPoseAndUpdate(objectPose);
            FramePose3D pose = new FramePose3D(handLockedFrame);
            pose.changeFrame(ReferenceFrame.getWorldFrame());
            handTransformToWorld.set(pose.getOrientation(), pose.getTranslation());
         }
         else
         {
            handLocked = false;
            affordancePoseLocked = false;
         }
      }
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      handPose.set(handTransformToWorld);

      graspFrame.update();
      preGraspFrames.update();
      postGraspFrames.update();

      gripperClosure[0] = interactableHand.getGripperClosure();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("HAND MENU");
      if (ImGui.radioButton(labels.get("RIGHT"), side == RobotSide.RIGHT))
      {
         side = RobotSide.RIGHT;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("LEFT"), side == RobotSide.LEFT))
      {
         side = RobotSide.LEFT;
      }
      if (side == RobotSide.LEFT)
      {
         //TODO add double hand logic with side dependent lists
         ImGui.pushStyleColor(ImGuiCol.Text, 1.0f, 0.0f, 0.0f, 1.0f);
         ImGui.text("Work in progress. Left hand currently not supported");
         ImGui.popStyleColor();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("ADD") + "##side"))
      {
         //TODO add double hand logic with side dependent lists
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("REMOVE") + "##side"))
      {
         //TODO add double hand logic with side dependent lists
      }
      ImGui.text("Hand configuration: ");
      if (ImGui.button(labels.get("OPEN")))
         interactableHand.openGripper();
      ImGui.sameLine();
      if (ImGui.button(labels.get("HALF_CLOSE")))
         interactableHand.setGripperToHalfClose();
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLOSE")))
         interactableHand.closeGripper();
      ImGui.sameLine();
      if (ImGui.button(labels.get("CRUSH")))
         interactableHand.crushGripper();
      if (ImGui.sliderFloat("SET CLOSURE", gripperClosure, interactableHand.getMinGripperClosure(), interactableHand.getMaxGripperClosure()))
         interactableHand.setGripperClosure(gripperClosure[0]);
      ImGui.separator();

      ImGui.text("PRE-GRASP MENU");
      ImGui.text("Pre-grasp Frames: ");
      ImGui.sameLine();
      preGraspFrames.renderImGuiWidgets(labels, "pregrasp");
      ImGui.separator();

      ImGui.text("GRASP MENU");
      ImGui.text("Grasp Frame: ");
      ImGui.sameLine();
      graspFrame.renderImGuiWidgets(labels, "grasp");
      ImGui.separator();

      ImGui.text("POST-GRASP MENU");
      ImGui.text("Post-grasp Frames: ");
      ImGui.sameLine();
      postGraspFrames.renderImGuiWidgets(labels, "postgrasp");
      boolean changedColor = false;
      if (handLocked)
      {
         ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
         changedColor = true;
      }
      if (ImGui.button(labels.get("LOCK HAND TO OBJECT")) && activeMenu[0] == RDXActiveAffordanceMenu.POST_GRASP)
         handLocked = !handLocked;
      if (changedColor)
         ImGui.popStyleColor();
      ImGui.separator();

      ImGui.text("PREVIOUS/NEXT FRAME: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("<"), 20, 25))
      {
         switch (activeMenu[0])
         {
            case PRE_GRASP ->
            {
               if (!preGraspFrames.isFirst())
                  preGraspFrames.selectPrevious();
            }
            case GRASP ->
            {
               if (preGraspFrames.getNumberOfFrames() > 0)
               {
                  activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
                  preGraspFrames.setSelectedIndexToSize();
                  preGraspFrames.selectPrevious();
               }
            }
            case POST_GRASP ->
            {
               if (!postGraspFrames.isFirst())
                  postGraspFrames.selectPrevious();
               else
               {
                  if (graspFrame.isSet())
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
                     graspFrame.selectFrame();
                  }
                  else
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
                     preGraspFrames.setSelectedIndexToSize();
                     preGraspFrames.selectPrevious();
                  }
               }
            }
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">"), 20, 25) || playing)
      {
         if (playing)
         {
            try
            {
               Thread.sleep(112); // about 9Hz
            }
            catch (InterruptedException e)
            {
            }
         }
         switch (activeMenu[0])
         {
            case PRE_GRASP ->
            {
               if (!preGraspFrames.isLast())
                  preGraspFrames.selectNext();
               else
               {
                  if (graspFrame.isSet())
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.GRASP;
                     graspFrame.selectFrame();
                  }
                  else
                  {
                     activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
                     postGraspFrames.resetSelectedIndex();
                     postGraspFrames.selectNext();
                  }
               }
            }
            case GRASP ->
            {
               if (postGraspFrames.getNumberOfFrames() > 0)
               {
                  activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
                  postGraspFrames.resetSelectedIndex();
                  postGraspFrames.selectNext();
               }
            }
            case POST_GRASP ->
            {
               if (!postGraspFrames.isLast())
                  postGraspFrames.selectNext();
               else
                  playing = false;
            }
         }
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("PLAY")))
      {
         playing = true;
      }
      ImGui.separator();

      if (ImGui.button("RESET"))
      {
         objectBuilder.getSelectedObject().resetToInitialPose();
         reset();
      }
      if (textInput.render())
      {
         fileName = textInput.getString();
      }
      if (ImGui.button("SAVE"))
      {
         saveToFile(fileName);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("LOAD")))
      {
         objectBuilder.reset();
         reset();
         loadFromFile(fileName);
      }
   }

   private void reset()
   {
      handTransformToWorld.setToZero();
      handTransformToWorld.getTranslation().set(-0.5, 0, 0);
      handTransformToWorld.getRotation().setYawPitchRoll(0.0, Math.toRadians(-90.0), 0.0);
      interactableHand.closeGripper();

      graspFrame.reset();
      preGraspFrames.reset();
      postGraspFrames.reset();
      activeMenu[0] = RDXActiveAffordanceMenu.NONE;
   }

   private ArrayList<Double> computeTrajectoryDurations(ArrayList<FramePose3D> preGraspPoses, FramePose3D graspPose, ArrayList<FramePose3D> postGraspPoses)
   {
      ArrayList<Double> trajectoryDurations = new ArrayList<>();
      int sizePreGrasp = preGraspPoses.size();
      // compute trajectory duration as the necessary duration required to reach two consecutive frames at the desired velocity
      for (int i = 0; i < sizePreGrasp; i++)
      {
         if (i != 0)
         {
            double positionDistance = preGraspPoses.get(i).getPositionDistance(preGraspPoses.get(i - 1));
            double angularDistance = preGraspPoses.get(i).getOrientationDistance(preGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      if (sizePreGrasp > 0)
      {
         double positionDistance = graspPose.getPositionDistance(preGraspPoses.get(sizePreGrasp - 1));
         double angularDistance = graspPose.getOrientationDistance(preGraspPoses.get(sizePreGrasp - 1));
         // take max duration required to achieve the desired linear velocity or angular velocity
         trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
      }
      else
         trajectoryDurations.add(DEFAULT_DURATION);

      for (int i = 0; i < postGraspPoses.size(); i++)
      {
         if (i != 0)
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(postGraspPoses.get(i - 1));
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(postGraspPoses.get(i - 1));
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else if (!graspFrame.isSet())
         {
            double positionDistance = postGraspPoses.get(i).getPositionDistance(graspPose);
            double angularDistance = postGraspPoses.get(i).getOrientationDistance(graspPose);
            // take max duration required to achieve the desired linear velocity or angular velocity
            trajectoryDurations.add(Math.max(positionDistance / LINEAR_VELOCITY, angularDistance / ANGULAR_VELOCITY));
         }
         else
            trajectoryDurations.add(DEFAULT_DURATION);
      }

      return trajectoryDurations;
   }

   public void saveToFile(String fileName)
   {
      // change affordance reference from world to initial object frame
      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
      affordanceFrame.changeParentFrame(initialObjectFrame);

      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory, fileName + ".json");
      if (file.isFileAccessAvailable())
      {
         LogTools.info("Saving to file ...");
         JSONFileTools.save(file, jsonNode ->
         {
            jsonNode.put("name", fileName);
            ArrayNode actionsArrayNode = jsonNode.putArray("actions");
            var preGraspPoses = preGraspFrames.getPoses();
            var graspPose = graspFrame.getPose();
            var postGraspPoses = postGraspFrames.getPoses();
            ArrayList<Double> trajectoryDurations = computeTrajectoryDurations(preGraspPoses, graspPose, postGraspPoses);
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
            for (int i = 0; i < preGraspPoses.size(); i++)
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               actionNode.put("side", side.getLowerCaseName());
               actionNode.put("trajectoryDuration", trajectoryDurations.get(i));
               preGraspPoses.get(i).changeFrame(affordanceFrame.getReferenceFrame());
               RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(i));
               JSONTools.toJSON(actionNode, transformToParent);

               double[] dataTrajectories = new double[16];
               transformToParent.get(dataTrajectories);
               csvDataMatrix.add(dataTrajectories);

               if (preGraspHandConfigurations.get(i) != null)
               {
                  dataTrajectories = new double[16];
                  for (int data = 0; data < dataTrajectories.length; data++)
                     dataTrajectories[data] = 0.0;
                  dataTrajectories[0] = HandConfiguration.valueOf(preGraspHandConfigurations.get(i).toString()).ordinal();
                  csvDataMatrix.add(dataTrajectories);

                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", side.getLowerCaseName());
                  extraActionNode.put("grip", preGraspHandConfigurations.get(i).toString());
               }
            }
            if (graspFrame.isSet())
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               actionNode.put("side", side.getLowerCaseName());
               actionNode.put("trajectoryDuration", trajectoryDurations.get(preGraspPoses.size()));
               graspPose.changeFrame(affordanceFrame.getReferenceFrame());
               RigidBodyTransform transformToParent = new RigidBodyTransform(graspPose);
               JSONTools.toJSON(actionNode, transformToParent);

               double[] dataTrajectories = new double[16];
               transformToParent.get(dataTrajectories);
               csvDataMatrix.add(dataTrajectories);

               if (graspFrame.getHandConfiguration() != null)
               {
                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", side.getLowerCaseName());
                  extraActionNode.put("grip", graspFrame.getHandConfiguration().toString());

                  dataTrajectories = new double[16];
                  for (int data = 0; data < dataTrajectories.length; data++)
                     dataTrajectories[data] = 0.0;
                  dataTrajectories[0] = HandConfiguration.valueOf(graspFrame.getHandConfiguration().toString()).ordinal();
                  csvDataMatrix.add(dataTrajectories);
               }
            }
            for (int i = 0; i < postGraspPoses.size(); i++)
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               actionNode.put("side", side.getLowerCaseName());
               actionNode.put("trajectoryDuration", trajectoryDurations.get(preGraspPoses.size() + 1 + i));
               postGraspPoses.get(i).changeFrame(affordanceFrame.getReferenceFrame());
               RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(i));
               JSONTools.toJSON(actionNode, transformToParent);

               double[] dataTrajectories = new double[16];
               transformToParent.get(dataTrajectories);
               csvDataMatrix.add(dataTrajectories);

               if (postGraspHandConfigurations.get(i) != null)
               {
                  dataTrajectories = new double[16];
                  for (int data = 0; data < dataTrajectories.length; data++)
                     dataTrajectories[data] = 0.0;
                  dataTrajectories[0] = HandConfiguration.valueOf(postGraspHandConfigurations.get(i).toString()).ordinal();
                  csvDataMatrix.add(dataTrajectories);

                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", side.getLowerCaseName());
                  extraActionNode.put("grip", postGraspHandConfigurations.get(i).toString());
               }
            }
         });
         LogTools.info("SAVED to file {}", file.getFileName());
      }
      else
      {
         LogTools.warn("Could not save to {}", file.getFileName());
      }

      WorkspaceResourceFile extraFile = new WorkspaceResourceFile(configurationsDirectory, fileName + "Extra.json");
      if (extraFile.isFileAccessAvailable())
      {
         JSONFileTools.save(extraFile, jsonNode ->
         {
            jsonNode.put("name", fileName);
            jsonNode.put("object", objectBuilder.getSelectedObjectName());
            JSONTools.toJSON(jsonNode, new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialPose()));
            ArrayNode framesArrayNode = jsonNode.putArray("frames");
            var preGraspObjectTransforms = preGraspFrames.getObjectTransforms();
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            jsonNode.put("numberPreGraspFrames", preGraspObjectTransforms.size());
            for (int i = 0; i < preGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, preGraspObjectTransforms.get(i));
               frameArray.put("grip", preGraspHandConfigurations.get(i) == null ? "" : preGraspHandConfigurations.get(i).toString());
            }
            if (graspFrame.isSet())
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, graspFrame.getObjectTransform());
               frameArray.put("grip", graspFrame.getHandConfiguration() == null ? "" : graspFrame.getHandConfiguration().toString());
            }
            var postGraspObjectTransforms = postGraspFrames.getObjectTransforms();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
            jsonNode.put("numberPostGraspFrames", postGraspObjectTransforms.size());
            for (int i = 0; i < postGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, postGraspObjectTransforms.get(i));
               frameArray.put("grip", postGraspHandConfigurations.get(i) == null ? "" : postGraspHandConfigurations.get(i).toString());
            }
         });
         LogTools.info("SAVED to file {}", extraFile.getFileName());
      }
      else
      {
         LogTools.warn("Could not save extra info to {}", extraFile.getFileName());
      }

      generateCSVFiles(fileName);
   }

   public void generateCSVFiles(String fileName)
   {
      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".csv");
      File csvFile = new File(filePath.toString());
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         for (int row = 0; row < csvDataMatrix.size(); row++)
         {
            double[] dataLine = csvDataMatrix.get(row);
            for (int col = 0; col < dataLine.length; col++)
            {
               writer.print(dataLine[col]);
               if (col < dataLine.length - 1)
                  writer.append(",");
            }
            if (row < csvDataMatrix.size() - 1)
            {
               writer.println();
            }
         }
         LogTools.info("SAVED to file {}", csvFile.getName());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void loadFromFile(String fileName)
   {
      Path filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + "Extra.json");
      final int[] preGraspFramesSize = new int[1];
      final int[] postGraspFramesSize = new int[1];
      if (Files.exists(filePath))
      {
         JSONFileTools.load(filePath, jsonNode ->
         {
            String objectName = jsonNode.get("object").asText();
            if (!objectName.isEmpty())
            {
               objectBuilder.loadObject(jsonNode.get("object").asText());
               RigidBodyTransform initialTransform = new RigidBodyTransform();
               JSONTools.toEuclid(jsonNode, initialTransform);
               objectBuilder.getSelectedObject().setPose(initialTransform);
            }
            preGraspFramesSize[0] = jsonNode.get("numberPreGraspFrames").asInt();
            postGraspFramesSize[0] = jsonNode.get("numberPostGraspFrames").asInt();
            JsonNode framesArrayNode = jsonNode.get("frames");
            for (int i = 0; i < preGraspFramesSize[0]; i++)
            {
               RigidBodyTransform preGraspObjectTransform = new RigidBodyTransform();
               JSONTools.toEuclid(framesArrayNode.get(i), preGraspObjectTransform);
               preGraspFrames.addObjectTransform(preGraspObjectTransform);
               String configuration = framesArrayNode.get(i).get("grip").asText();
               preGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
            }
            RigidBodyTransform graspObjectTransform = new RigidBodyTransform();
            JSONTools.toEuclid(framesArrayNode.get(preGraspFramesSize[0]), graspObjectTransform);
            graspFrame.setObjectTransform(graspObjectTransform);
            String configuration = framesArrayNode.get(preGraspFramesSize[0]).get("grip").asText();
            graspFrame.setHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
            for (int i = 0; i < postGraspFramesSize[0]; i++)
            {
               RigidBodyTransform postGraspObjectTransform = new RigidBodyTransform();
               JSONTools.toEuclid(framesArrayNode.get(i + preGraspFramesSize[0] + 1), postGraspObjectTransform);
               postGraspFrames.addObjectTransform(postGraspObjectTransform);
               configuration = framesArrayNode.get(i + preGraspFramesSize[0] + 1).get("grip").asText();
               postGraspFrames.addHandConfiguration(configuration.isEmpty() ? null : HandConfiguration.valueOf(configuration));
            }
         });
         LogTools.info("LOADED file {}", filePath);
      }
      else
      {
         LogTools.warn("Could not load file {}", filePath);
      }

      // change affordance reference from whatever it is now to loaded initial object pose
      RigidBodyTransform initialObjectTransform = new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialTransformToWorld());
      initialObjectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(), initialObjectTransform);
      affordanceFrame.changeParentFrame(initialObjectFrame);

      filePath = Paths.get(configurationsDirectory.getFilesystemDirectory().toString(), fileName + ".json");
      if (Files.exists(filePath))
      {
         JSONFileTools.load(filePath, jsonNode ->
         {
            JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
            {
               String actionType = actionNode.get("type").asText();
               if (actionType.equals("RDXHandPoseAction"))
               {
                  //                  side = RobotSide.getSideFromString(jsonNode.get("side").asText());
                  RigidBodyTransform frameTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(actionNode, frameTransform);
                  if (preGraspFrames.getNumberOfFrames() < preGraspFramesSize[0])
                     preGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
                  else if (!graspFrame.isSet())
                     graspFrame.setFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
                  else
                     postGraspFrames.addFrame(new FramePose3D(affordanceFrame.getReferenceFrame(), frameTransform));
               }
            });
         });
         LogTools.info("LOADED file {}", filePath);
      }
      else
      {
         LogTools.warn("Could not load file {}", filePath);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graspFrame.getRenderables(renderables, pool);
      preGraspFrames.getRenderables(renderables, pool);
      postGraspFrames.getRenderables(renderables, pool);
   }

   public static void main(String[] args)
   {
      new RDXAffordanceEditorUI();
   }
}