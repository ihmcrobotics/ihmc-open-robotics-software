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
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.io.*;

import java.util.*;

public class RDXAffordanceEditorUI
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXInteractableSakeGripper interactableHand;
   private final RigidBodyTransform handTransformToWorld = new RigidBodyTransform();
   private final FramePose3D handPose = new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformToWorld);
   private RDXInteractableObjectBuilder objectBuilder;
   private final PoseReferenceFrame objectFrame = new PoseReferenceFrame("objectFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame affordanceFrame = new PoseReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private final float[] gripperClosure = new float[1];
   // affordance poses
   private RDXAffordanceFrame graspFrame;
   private RDXAffordanceFrames preGraspFrames;
   private RDXAffordanceFrames postGraspFrames;

   public boolean affordancePoseLocked = false;
   private boolean handLocked = false;
   private PoseReferenceFrame handLockedFrame;
   private RDXActiveAffordanceMenu[] activeMenu;

   private final ImGuiInputText textInput = new ImGuiInputText("File name to save/load");
   private String fileName = "";
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/affordances");

   public RDXAffordanceEditorUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            objectBuilder = new RDXInteractableObjectBuilder(baseUI);
            baseUI.getImGuiPanelManager().addPanel(objectBuilder.getWindowName(), objectBuilder::renderImGuiWidgets);

            handTransformToWorld.getTranslation().set(-0.5, 0, 0);
            interactableHand = new RDXInteractableSakeGripper(baseUI.getPrimary3DPanel(), handTransformToWorld);

            activeMenu = new RDXActiveAffordanceMenu[1];
            activeMenu[0] = RDXActiveAffordanceMenu.PRE_GRASP;
            preGraspFrames = new RDXAffordanceFrames(interactableHand,
                                                     handTransformToWorld,
                                                     handPose,
                                                     objectBuilder.getSelectedObject().getTransformToWorld(),
                                                     affordanceFrame,
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
                                                affordanceFrame,
                                                activeMenu,
                                                Color.BLACK);
            activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
            postGraspFrames = new RDXAffordanceFrames(interactableHand,
                                                      handTransformToWorld,
                                                      handPose,
                                                      objectBuilder.getSelectedObject().getTransformToWorld(),
                                                      affordanceFrame,
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
               handPose.changeFrame(affordanceFrame);
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
      handPose.changeFrame(affordanceFrame);

      graspFrame.update();
      preGraspFrames.update();
      postGraspFrames.update();

      gripperClosure[0] = interactableHand.getGripperClosure();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("HAND MENU");
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
      interactableHand.closeGripper();
      graspFrame.reset();
      preGraspFrames.reset();
      postGraspFrames.reset();
      activeMenu[0] = RDXActiveAffordanceMenu.NONE;
   }

   public void saveToFile(String fileName)
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory, fileName + ".json");
      if (file.isFileAccessAvailable())
      {
         LogTools.info("Saving to file ...");
         JSONFileTools.save(file, root ->
         {
            root.put("name", fileName);
            ArrayNode actionsArrayNode = root.putArray("actions");
            var preGraspPoses = preGraspFrames.getPoses();
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            for (int i = 0; i < preGraspPoses.size(); i++)
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               //TODO side-> Robot Side add radio button for inserting and editing right or left hand
               actionNode.put("side", "right");
               //TODO fix trajectory duration to a good cm/s velocity
               actionNode.put("trajectoryDuration", 1);
               RigidBodyTransform transformToParent = new RigidBodyTransform(preGraspPoses.get(i));
               JSONTools.toJSON(actionNode, transformToParent);

               if (preGraspHandConfigurations.get(i) != null)
               {
                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", "right");
                  extraActionNode.put("grip", preGraspHandConfigurations.get(i).toString());
               }
            }
            if (graspFrame.isSet())
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               actionNode.put("side", "right");
               actionNode.put("trajectoryDuration", 1);
               RigidBodyTransform transformToParent = new RigidBodyTransform(graspFrame.getPose());
               JSONTools.toJSON(actionNode, transformToParent);
               if (graspFrame.getHandConfiguration() != null)
               {
                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", "right");
                  extraActionNode.put("grip", graspFrame.getHandConfiguration().toString());
               }
            }

            var postGraspPoses = postGraspFrames.getPoses();
            var postGraspHandConfigurations = postGraspFrames.getHandConfigurations();
            for (int i = 0; i < postGraspPoses.size(); i++)
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", "RDXHandPoseAction");
               actionNode.put("parentFrame", objectBuilder.getSelectedObjectName());
               actionNode.put("side", "right");
               actionNode.put("trajectoryDuration", 1);
               RigidBodyTransform transformToParent = new RigidBodyTransform(postGraspPoses.get(i));
               JSONTools.toJSON(actionNode, transformToParent);

               if (postGraspHandConfigurations.get(i) != null)
               {
                  ObjectNode extraActionNode = actionsArrayNode.addObject();
                  extraActionNode.put("type", "RDXHandConfigurationAction");
                  extraActionNode.put("side", "right");
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
         JSONFileTools.save(extraFile, root ->
         {
            root.put("name", fileName);
            root.put("object", objectBuilder.getSelectedObjectName());
            JSONTools.toJSON(root, new RigidBodyTransform(objectBuilder.getSelectedObject().getInitialPose()));
            ArrayNode framesArrayNode = root.putArray("frames");
            var preGraspObjectTransforms = preGraspFrames.getObjectTransforms();
            var preGraspHandConfigurations = preGraspFrames.getHandConfigurations();
            root.put("numberPreGraspFrames", preGraspObjectTransforms.size());
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
            root.put("numberPostGraspFrames", postGraspObjectTransforms.size());
            for (int i = 0; i < postGraspObjectTransforms.size(); i++)
            {
               ObjectNode frameArray = framesArrayNode.addObject();
               JSONTools.toJSON(frameArray, postGraspObjectTransforms.get(i));
               frameArray.put("grip", postGraspHandConfigurations.get(i) == null ? "" : postGraspHandConfigurations.get(i).toString());
            }
         });
         LogTools.info("SAVED extra info to file {}", extraFile.getFileName());
      }
      else
      {
         LogTools.warn("Could not save extra info to {}", extraFile.getFileName());
      }
   }

   public void loadFromFile(String fileName)
   {
      WorkspaceResourceFile extraFile = new WorkspaceResourceFile(configurationsDirectory, "/affordances/" + fileName + "Extra.json");
      LogTools.info("Loading from {}", extraFile.getFileName());
      final int[] preGraspFramesSize = new int[1];
      final int[] postGraspFramesSize = new int[1];
      if (extraFile.isFileAccessAvailable())
      {
         JSONFileTools.load(extraFile, root ->
         {
            String objectName = root.get("object").asText();
            if (!objectName.isEmpty())
            {
               objectBuilder.loadObject(root.get("object").asText());
               RigidBodyTransform initialTransform = new RigidBodyTransform();
               JSONTools.toEuclid(root, initialTransform);
               objectBuilder.getSelectedObject().setInitialPose(initialTransform);
            }
            preGraspFramesSize[0] = root.get("numberPreGraspFrames").asInt();
            postGraspFramesSize[0] = root.get("numberPostGraspFrames").asInt();
            JsonNode framesArrayNode = root.get("frames");
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
      }
      else
      {
         LogTools.warn("Could not load extra info from {}" + extraFile.getFileName());
      }

      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory, "/affordances/" + fileName + ".json");

      LogTools.info("Loading from {}", file.getClasspathResource().toString());
      if (file.isFileAccessAvailable())
      {
         JSONFileTools.load(file, jsonNode ->
         {
            JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
            {
               String actionType = actionNode.get("type").asText();
               if (actionType.equals("RDXHandPoseAction"))
               {
                  //                  side = RobotSide.getSideFromString(jsonNode.get("side").asText());
                  //                  trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
                  RigidBodyTransform frameTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(actionNode, frameTransform);
                  if (preGraspFrames.getNumberOfFrames() < preGraspFramesSize[0])
                     preGraspFrames.addFrame(new FramePose3D(affordanceFrame, frameTransform));
                  else if (!graspFrame.isSet())
                     graspFrame.setFrame(new FramePose3D(affordanceFrame, frameTransform));
                  else
                     postGraspFrames.addFrame(new FramePose3D(affordanceFrame, frameTransform));
               }
            });
         });
      }
      else
      {
         LogTools.warn("Could not load info from {}" + file.getFileName());
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
