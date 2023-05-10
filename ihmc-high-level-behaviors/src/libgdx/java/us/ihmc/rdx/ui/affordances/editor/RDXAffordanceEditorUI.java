package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableSakeGripper;
import us.ihmc.rdx.ui.interactable.RDXInteractableObjectBuilder;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.nio.file.Path;
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
   private RDXAffordanceFrame graspPose;
   private RDXAffordanceFrames preGraspPoses;
   private RDXAffordanceFrames postGraspPoses;

   public boolean affordancePoseLocked = false;
   private boolean handLocked = false;
   private PoseReferenceFrame handLockedFrame;
   private RDXActiveAffordanceMenu[] activeMenu;
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/boxAffordance");

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
            preGraspPoses = new RDXAffordanceFrames(interactableHand,
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
            graspPose = new RDXAffordanceFrame(interactableHand,
                                               handTransformToWorld,
                                               handPose,
                                               objectBuilder.getSelectedObject().getTransformToWorld(),
                                               affordanceFrame,
                                               activeMenu,
                                               Color.BLACK);
            activeMenu[0] = RDXActiveAffordanceMenu.POST_GRASP;
            postGraspPoses = new RDXAffordanceFrames(interactableHand,
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

      graspPose.update();
      preGraspPoses.update();
      postGraspPoses.update();

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
      preGraspPoses.renderImGuiWidgets(labels, "pregrasp");
      ImGui.separator();

      ImGui.text("GRASP MENU");
      ImGui.text("Grasp Frame: ");
      ImGui.sameLine();
      graspPose.renderImGuiWidgets(labels, "grasp");
      ImGui.separator();

      ImGui.text("POST-GRASP MENU");
      ImGui.text("Post-grasp Frames: ");
      ImGui.sameLine();
      postGraspPoses.renderImGuiWidgets(labels, "postgrasp");
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
         handTransformToWorld.setToZero();
         handTransformToWorld.getTranslation().set(-0.5, 0, 0);
         interactableHand.closeGripper();
         objectBuilder.getSelectedObject().resetToInitialPose();
         graspPose.reset();
         preGraspPoses.reset();
         postGraspPoses.reset();
         activeMenu[0] = RDXActiveAffordanceMenu.NONE;
      }
      if (ImGui.button("Save to json"))
      {
         saveToJSON();
      }

      if (ImGui.button(labels.get("Load all affordance points from json")))
      {
         loadAllFromJSON("box");
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graspPose.getRenderables(renderables, pool);
      preGraspPoses.getRenderables(renderables, pool);
      postGraspPoses.getRenderables(renderables, pool);
   }

   public void loadAllFromJSON(String objectName)
   {
      WorkspaceFile file = new WorkspaceFile(configurationsDirectory, objectName + "Affordance.json");
      Path filePath = file.getFilesystemFile();
      JSONFileTools.load(filePath, jsonNode ->
      {
         //         preGraspPoses.clear();
         //         preGraspFrames.clear();
         //         preGraspIndices.clear();
         //         preGraspPoseGraphics.clear();
         //         preGraspColorIndex = 0;
         Iterator<Map.Entry<String, JsonNode>> it = jsonNode.fields();

         Map.Entry<String, JsonNode> map = it.next();
         while (true)
         {
            String affordanceName = map.getKey();
            JsonNode node = map.getValue();
            double x = node.get("x").asDouble();
            double y = node.get("y").asDouble();
            double z = node.get("z").asDouble();
            double roll = node.get("roll").asDouble();
            double pitch = node.get("pitch").asDouble();
            double yaw = node.get("yaw").asDouble();

            if (affordanceName.contains("grasp"))
            {
               //               initialPose = new FramePose3D(affordanceFrame);
               //               initialPose.set(x, y, z, yaw, pitch, roll);
               //               initialPose.changeFrame(ReferenceFrame.getWorldFrame());
               //               initialPoseGraphic.updateFromFramePose(initialPose);
            }
            else if (affordanceName.contains("press"))
            {
               //               graspPose = new FramePose3D(affordanceFrame);
               //               graspPose.set(x, y, z, yaw, pitch, roll);
               //               graspPose.changeFrame(ReferenceFrame.getWorldFrame());
               //               graspPoseGraphic.updateFromFramePose(graspPose);
            }
            else
            {
               //               preGraspIndices.add(preGraspIndex);
               //               FramePose3D pose = new FramePose3D(affordanceFrame);
               //               pose.set(x, y, z, yaw, pitch, roll);
               //               pose.changeFrame(ReferenceFrame.getWorldFrame());
               //               preGraspPoses.add(pose);
               //               preGraspPoseGraphics.add(new RDXReferenceFrameGraphic(0.3, preGraspColors.get(preGraspColorIndex % preGraspColors.size())));
               //               preGraspColorIndex++;
               //               PoseReferenceFrame frame = new PoseReferenceFrame(preGraspIndex + "Frame", affordanceFrame);
               //               pose.changeFrame(affordanceFrame);
               //               frame.setPoseAndUpdate(pose);
               //               preGraspFrames.add(frame);
            }

            if (it.hasNext())
               map = it.next();
            else
               break;
         }
      });
   }

   public void loadFromJSON(FramePose3DReadOnly objectPose, String objectName, String affordanceName)
   {
      WorkspaceFile file = new WorkspaceFile(configurationsDirectory, objectName + "Affordance.json");
      Path filePath = file.getFilesystemFile();
      JSONFileTools.load(filePath, jsonNode ->
      {
         JsonNode node = jsonNode.get(affordanceName);
         if (objectPose.getReferenceFrame().toString() != node.get("referenceFrame").asText())
         {
            // ref do not match
         }

         // this is pose w.r.t object
         double x = node.get("x").asDouble();
         double y = node.get("y").asDouble();
         double z = node.get("z").asDouble();
         double roll = node.get("roll").asDouble();
         double pitch = node.get("pitch").asDouble();
         double yaw = node.get("yaw").asDouble();

         if (affordanceName.contains("grasp"))
         {
            //            initialPose = new FramePose3D(affordanceFrame);
            //            initialPose.set(x, y, z, yaw, pitch, roll);
            //            initialPose.changeFrame(ReferenceFrame.getWorldFrame());
            //            initialPoseGraphic.updateFromFramePose(initialPose);
         }

         else if (affordanceName.contains("press"))
         {
            //            graspPose = new FramePose3D(affordanceFrame);
            //            graspPose.set(x, y, z, yaw, pitch, roll);
            //            graspPose.changeFrame(ReferenceFrame.getWorldFrame());
            //            graspPoseGraphic.updateFromFramePose(graspPose);
         }
      });
   }

   public void saveToJSON()
   {
      WorkspaceFile file = new WorkspaceFile(configurationsDirectory, "boxAffordance.json");
      if (file.isFileAccessAvailable())
      {
         LogTools.info("saving affordance to json");
         JSONFileTools.save(file, root ->
         {
            // affordance 1
            ObjectNode pregraspNode = root.putObject("pregraspPoint");

            //            FramePose3D handPose = new FramePose3D(initialFrame);
            //            handPose.changeFrame(affordanceFrame);

            pregraspNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            pregraspNode.put("x", handPose.getPosition().getX());
            pregraspNode.put("y", handPose.getPosition().getY());
            pregraspNode.put("z", handPose.getPosition().getZ());
            pregraspNode.put("roll", handPose.getOrientation().getRoll());
            pregraspNode.put("pitch", handPose.getOrientation().getPitch());
            pregraspNode.put("yaw", handPose.getOrientation().getYaw());

            // affordance 2
            ObjectNode graspNode = root.putObject("graspPoint");

            //            handPose = new FramePose3D(graspFrame);
            //            handPose.changeFrame(affordanceFrame);

            graspNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            graspNode.put("x", handPose.getPosition().getX());
            graspNode.put("y", handPose.getPosition().getY());
            graspNode.put("z", handPose.getPosition().getZ());
            graspNode.put("roll", handPose.getOrientation().getRoll());
            graspNode.put("pitch", handPose.getOrientation().getPitch());
            graspNode.put("yaw", handPose.getOrientation().getYaw());

            //            for (int i = 0; i < preGraspIndices.size(); ++i)
            //            {
            //               ObjectNode customNode = root.putObject(preGraspIndices.get(i).toString());
            ////               handPose = new FramePose3D(preGraspFrames.get(i));
            ////               handPose.changeFrame(affordanceFrame);
            //               customNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            //               customNode.put("x", handPose.getPosition().getX());
            //               customNode.put("y", handPose.getPosition().getY());
            //               customNode.put("z", handPose.getPosition().getZ());
            //               customNode.put("roll", handPose.getOrientation().getRoll());
            //               customNode.put("pitch", handPose.getOrientation().getPitch());
            //               customNode.put("yaw", handPose.getOrientation().getYaw());
            //            }
         });
         LogTools.info("SAVED affordance to json");
      }
      else
      {
         LogTools.warn("Could not write to " + file);
      }
   }

   public static void main(String[] args)
   {
      new RDXAffordanceEditorUI();
   }
}
