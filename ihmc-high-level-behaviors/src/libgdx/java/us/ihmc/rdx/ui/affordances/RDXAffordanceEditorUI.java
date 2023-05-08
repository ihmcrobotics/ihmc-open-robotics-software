package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImFont;
import imgui.ImFontAtlas;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.type.ImDouble;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableDummyHand;
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
   private RDXInteractableDummyHand interactableHand;
   private RigidBodyTransform handTransformToWorld = new RigidBodyTransform();
   private final FramePose3D handPose = new FramePose3D(ReferenceFrame.getWorldFrame(), handTransformToWorld);
   private RDXInteractableObjectBuilder objectBuilder;
   private RigidBodyTransform objectToWorld = new RigidBodyTransform();
   private final PoseReferenceFrame affordanceFrame = new PoseReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   // initial pose
   private FramePose3D initialPose = new FramePose3D();
   private boolean isInitialPoseSet = false;
   private RDXReferenceFrameGraphic initialPoseGraphic;
   private final PoseReferenceFrame initialFrame = new PoseReferenceFrame("initialFrame", affordanceFrame);
   private HandConfiguration initialHandConfiguration;
   // grasp pose
   private FramePose3D graspPose = new FramePose3D();
   private boolean isGraspPoseSet = false;
   private RDXReferenceFrameGraphic graspPoseGraphic;
   private final PoseReferenceFrame graspFrame = new PoseReferenceFrame("graspFrame", affordanceFrame);
   private final float[] gripperClosure = new float[1];
   private HandConfiguration graspHandConfiguration;
   // pre-grasp poses
   private final ArrayList<FramePose3D> preGraspPoses = new ArrayList<>();
   private final ArrayList<PoseReferenceFrame> preGraspFrames = new ArrayList<>();
   private final ArrayList<RDXReferenceFrameGraphic> preGraspPoseGraphics = new ArrayList<>();
   private final ArrayList<Color> preGraspColors = new ArrayList<>(Arrays.asList(new Color(0xFFE4B5FF),
                                                                                 new Color(0xFF8C00FF),
                                                                                 new Color(0xFFDAB9FF),
                                                                                 new Color(0xFF6600FF),
                                                                                 new Color(0xFFA07AFF)));
   private final ArrayList<Integer> preGraspIndices = new ArrayList<>();
   private int preGraspIndex = 0;
   private int preGraspColorIndex = 0;
   private final ArrayList<HandConfiguration> preGraspHandConfigurations = new ArrayList<>();
   // post-grasp poses
   private final ArrayList<FramePose3D> postGraspPoses = new ArrayList<>();
   private final ArrayList<PoseReferenceFrame> postGraspFrames = new ArrayList<>();
   private final ArrayList<RDXReferenceFrameGraphic> postGraspPoseGraphics = new ArrayList<>();
   private final ArrayList<Color> postGraspColors = new ArrayList<>(Arrays.asList(new Color(0xD8BFD8FF),
                                                                                  new Color(0xBA55D3FF),
                                                                                  new Color(0x9932CCFF),
                                                                                  new Color(0x8A2BE2FF),
                                                                                  new Color(0x4B0082FF)));
   private final ArrayList<Integer> postGraspIndices = new ArrayList<>();
   private int postGraspIndex = 0;
   private int postGraspColorIndex = 0;
   private final ArrayList<HandConfiguration> postGraspHandConfigurations = new ArrayList<>();

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

            initialPoseGraphic = new RDXReferenceFrameGraphic(0.1, Color.WHITE);
            graspPoseGraphic = new RDXReferenceFrameGraphic(0.1, Color.BLACK);

            handTransformToWorld.getTranslation().set(-0.5, 0, 0);
            interactableHand = new RDXInteractableDummyHand(baseUI.getPrimary3DPanel(), handTransformToWorld);
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
         FramePose3D objectPose = new FramePose3D(objectBuilder.getSelectedObject().getObjectFrame());
         objectPose.changeFrame(ReferenceFrame.getWorldFrame());
         affordanceFrame.setPoseAndUpdate(objectPose);
      }
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      handPose.set(handTransformToWorld);
      handPose.changeFrame(affordanceFrame);

      initialPose = new FramePose3D(initialFrame);
      initialPose.changeFrame(ReferenceFrame.getWorldFrame());
      initialPoseGraphic.updateFromFramePose(initialPose);

      graspPose = new FramePose3D(graspFrame);
      graspPose.changeFrame(ReferenceFrame.getWorldFrame());
      graspPoseGraphic.updateFromFramePose(graspPose);

      for (int i = 0; i < preGraspPoses.size(); ++i)
      {
         preGraspPoses.set(i, new FramePose3D(preGraspFrames.get(i)));
         preGraspPoses.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         preGraspPoseGraphics.get(i).updateFromFramePose(preGraspPoses.get(i));
      }

      for (int i = 0; i < postGraspPoses.size(); ++i)
      {
         postGraspPoses.set(i, new FramePose3D(postGraspFrames.get(i)));
         postGraspPoses.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         postGraspPoseGraphics.get(i).updateFromFramePose(postGraspPoses.get(i));
      }

      gripperClosure[0] = interactableHand.getGripperClosure();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("HAND MENU");
      ImGui.text("Hand configuration: ");
      if (ImGui.button(labels.get("OPEN")))
         interactableHand.openGripper();
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLOSE")))
         interactableHand.closeGripper();
      ImGui.sameLine();
      if (ImGui.button(labels.get("NEUTRAL")))
         interactableHand.setGripperToNeutral();
      if (ImGui.sliderFloat("SET CLOSURE",
                            gripperClosure, interactableHand.getMinGripperClosure(), interactableHand.getMaxGripperClosure()))
         interactableHand.setGripperClosure(gripperClosure[0]);
      ImGui.separator();

      ImGui.text("PRE-GRASP MENU");
      ImGui.text("Initial Frame: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##initial"))
      {
         isInitialPoseSet = true;
         initialFrame.setPoseAndUpdate(handPose);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("DELETE") + "##initial"))
      {
         isInitialPoseSet = false;
         initialHandConfiguration = null;
      }
      ImGui.sameLine();
      ImGui.text("|");
      ImGui.sameLine();
      if (isInitialPoseSet)
         ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
      else
         ImGui.pushStyleColor(ImGuiCol.Button, 1.0f, 0.0f, 0.0f, 1.0f);
      if (ImGui.button(labels.get("TELEPORT") + "##initial"))
      {
         if (isInitialPoseSet)
            handTransformToWorld.set(initialPose);  // move hand to pregrasp point
      }
      ImGui.popStyleColor();
      if (ImGui.button(labels.get("SET HAND CONFIGURATION") + "##initial"))
      {
         initialHandConfiguration = interactableHand.getConfiguration();
      }

      ImGui.text("Other Frames: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("ADD") + "##pregrasp"))
      {
         preGraspIndex++;
         PoseReferenceFrame frame = new PoseReferenceFrame(preGraspIndex + "Frame", affordanceFrame);
         frame.setPoseAndUpdate(handPose);

         preGraspIndices.add(preGraspIndex);
         preGraspPoses.add(handPose);
         preGraspFrames.add(frame);
         preGraspPoseGraphics.add(new RDXReferenceFrameGraphic(0.1, preGraspColors.get(preGraspColorIndex % preGraspColors.size())));
         preGraspColorIndex++;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##pregrasp"))
      {
         preGraspIndices.clear();
         preGraspFrames.clear();
         preGraspPoses.clear();
         preGraspPoseGraphics.clear();
         preGraspColorIndex = 0;
      }
      if (preGraspIndices.size() > 0)
      {
         for (int i = 0; i < preGraspIndices.size(); ++i)
         {
            if (i % 5 != 0)
               ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
            if (ImGui.button(labels.get(preGraspIndices.get(i).toString()) + "##pregrasp"))
            {
               // move hand to selected frame
               FramePose3D selectedPose = preGraspPoses.get(i);
               handTransformToWorld.set(selectedPose);
            }
            ImGui.popStyleColor();
            ImGui.sameLine();
            // handle the delete button click event here...
            String xButtonLabel = "pregrasp" + i;
            if (ImGui.button(labels.get("X") + "##" + xButtonLabel, 15, 15))
            {
               preGraspFrames.remove(i);
               preGraspPoses.remove(i);
               preGraspPoseGraphics.remove(i);
               preGraspIndices.remove(i);
            }
         }
      }
      else
      {
         preGraspColorIndex = 0;
         preGraspIndex = 0;
      }
      ImGui.separator();

      ImGui.text("GRASP MENU");
      ImGui.text("Grasp Frame: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("SET") + "##grasp"))
      {
         isGraspPoseSet = true;
         graspFrame.setPoseAndUpdate(handPose);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("DELETE") + "##grasp"))
      {
         isGraspPoseSet = false;
      }
      ImGui.sameLine();
      ImGui.text("|");
      ImGui.sameLine();
      if (isGraspPoseSet)
         ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
      else
         ImGui.pushStyleColor(ImGuiCol.Button, 1.0f, 0.0f, 0.0f, 1.0f);
      if (ImGui.button(labels.get("TELEPORT") + "##grasp"))
      {
         if (isGraspPoseSet)
            handTransformToWorld.set(graspPose); // move hand to grasp point
      }
      ImGui.popStyleColor();
      ImGui.separator();

      ImGui.text("POST-GRASP MENU");
      ImGui.text("Post-grasp Frames: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("ADD") + "##postgrasp"))
      {
         postGraspIndex++;
         PoseReferenceFrame frame = new PoseReferenceFrame(postGraspIndex + "Frame", affordanceFrame);
         frame.setPoseAndUpdate(handPose);

         postGraspIndices.add(postGraspIndex);
         postGraspPoses.add(handPose);
         postGraspFrames.add(frame);
         postGraspPoseGraphics.add(new RDXReferenceFrameGraphic(0.1, postGraspColors.get(postGraspColorIndex % postGraspColors.size())));
         postGraspColorIndex++;
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("CLEAR ALL") + "##postgrasp"))
      {
         postGraspIndices.clear();
         postGraspFrames.clear();
         postGraspPoses.clear();
         postGraspPoseGraphics.clear();
         postGraspColorIndex = 0;
      }
      if (postGraspIndices.size() > 0)
      {
         for (int i = 0; i < postGraspIndices.size(); ++i)
         {
            if (i % 5 != 0)
               ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 1.0f, 0.0f, 1.0f);
            if (ImGui.button(labels.get(postGraspIndices.get(i).toString()) + "##postgrasp"))
            {
               // move hand to selected frame
               FramePose3D selectedPose = postGraspPoses.get(i);
               handTransformToWorld.set(selectedPose);
            }
            ImGui.popStyleColor();
            ImGui.sameLine();
            // handle the delete button click event here...
            String xButtonLabel = "postgrasp" + i;
            if (ImGui.button(labels.get("X") + "##" + xButtonLabel, 15, 15))
            {
               postGraspFrames.remove(i);
               postGraspPoses.remove(i);
               postGraspPoseGraphics.remove(i);
               postGraspIndices.remove(i);
            }
         }
      }
      else
      {
         postGraspColorIndex = 0;
         postGraspIndex = 0;
      }
      ImGui.separator();

      if (ImGui.button("RESET"))
      {
         handTransformToWorld.setToZero();
         handTransformToWorld.getTranslation().set(-0.5, 0, 0);
         interactableHand.setGripperToNeutral();
         objectBuilder.getSelectedObject().resetPose();
         isInitialPoseSet = false;
         isGraspPoseSet = false;
         preGraspIndices.clear();
         preGraspFrames.clear();
         preGraspPoses.clear();
         preGraspPoseGraphics.clear();
         preGraspColorIndex = 0;
         postGraspIndices.clear();
         postGraspFrames.clear();
         postGraspPoses.clear();
         postGraspPoseGraphics.clear();
         postGraspColorIndex = 0;
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
      if (isInitialPoseSet)
         initialPoseGraphic.getRenderables(renderables, pool);
      if (isGraspPoseSet)
         graspPoseGraphic.getRenderables(renderables, pool);
      for (RDXReferenceFrameGraphic preGraspPoseGraphic : preGraspPoseGraphics)
      {
         preGraspPoseGraphic.getRenderables(renderables, pool);
      }
      for (RDXReferenceFrameGraphic postGraspPoseGraphic : postGraspPoseGraphics)
      {
         postGraspPoseGraphic.getRenderables(renderables, pool);
      }
   }

   public void loadAllFromJSON(String objectName)
   {
      WorkspaceFile file = new WorkspaceFile(configurationsDirectory, objectName + "Affordance.json");
      Path filePath = file.getFilesystemFile();
      JSONFileTools.load(filePath, jsonNode ->
      {
         preGraspPoses.clear();
         preGraspFrames.clear();
         preGraspIndices.clear();
         preGraspPoseGraphics.clear();
         preGraspColorIndex = 0;
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
               initialPose = new FramePose3D(affordanceFrame);
               initialPose.set(x, y, z, yaw, pitch, roll);
               initialPose.changeFrame(ReferenceFrame.getWorldFrame());
               initialPoseGraphic.updateFromFramePose(initialPose);
            }
            else if (affordanceName.contains("press"))
            {
               graspPose = new FramePose3D(affordanceFrame);
               graspPose.set(x, y, z, yaw, pitch, roll);
               graspPose.changeFrame(ReferenceFrame.getWorldFrame());
               graspPoseGraphic.updateFromFramePose(graspPose);
            }
            else
            {
               preGraspIndices.add(preGraspIndex);
               FramePose3D pose = new FramePose3D(affordanceFrame);
               pose.set(x, y, z, yaw, pitch, roll);
               pose.changeFrame(ReferenceFrame.getWorldFrame());
               preGraspPoses.add(pose);
               preGraspPoseGraphics.add(new RDXReferenceFrameGraphic(0.3, preGraspColors.get(preGraspColorIndex % preGraspColors.size())));
               preGraspColorIndex++;
               PoseReferenceFrame frame = new PoseReferenceFrame(preGraspIndex + "Frame", affordanceFrame);
               pose.changeFrame(affordanceFrame);
               frame.setPoseAndUpdate(pose);
               preGraspFrames.add(frame);
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
            initialPose = new FramePose3D(affordanceFrame);
            initialPose.set(x, y, z, yaw, pitch, roll);
            initialPose.changeFrame(ReferenceFrame.getWorldFrame());
            initialPoseGraphic.updateFromFramePose(initialPose);
         }

         else if (affordanceName.contains("press"))
         {
            graspPose = new FramePose3D(affordanceFrame);
            graspPose.set(x, y, z, yaw, pitch, roll);
            graspPose.changeFrame(ReferenceFrame.getWorldFrame());
            graspPoseGraphic.updateFromFramePose(graspPose);
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

            FramePose3D handPose = new FramePose3D(initialFrame);
            handPose.changeFrame(affordanceFrame);

            pregraspNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            pregraspNode.put("x", handPose.getPosition().getX());
            pregraspNode.put("y", handPose.getPosition().getY());
            pregraspNode.put("z", handPose.getPosition().getZ());
            pregraspNode.put("roll", handPose.getOrientation().getRoll());
            pregraspNode.put("pitch", handPose.getOrientation().getPitch());
            pregraspNode.put("yaw", handPose.getOrientation().getYaw());

            // affordance 2
            ObjectNode graspNode = root.putObject("graspPoint");

            handPose = new FramePose3D(graspFrame);
            handPose.changeFrame(affordanceFrame);

            graspNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            graspNode.put("x", handPose.getPosition().getX());
            graspNode.put("y", handPose.getPosition().getY());
            graspNode.put("z", handPose.getPosition().getZ());
            graspNode.put("roll", handPose.getOrientation().getRoll());
            graspNode.put("pitch", handPose.getOrientation().getPitch());
            graspNode.put("yaw", handPose.getOrientation().getYaw());

            for (int i = 0; i < preGraspIndices.size(); ++i)
            {
               ObjectNode customNode = root.putObject(preGraspIndices.get(i).toString());
               handPose = new FramePose3D(preGraspFrames.get(i));
               handPose.changeFrame(affordanceFrame);
               customNode.put("referenceFrame", handPose.getReferenceFrame().toString());
               customNode.put("x", handPose.getPosition().getX());
               customNode.put("y", handPose.getPosition().getY());
               customNode.put("z", handPose.getPosition().getZ());
               customNode.put("roll", handPose.getOrientation().getRoll());
               customNode.put("pitch", handPose.getOrientation().getPitch());
               customNode.put("yaw", handPose.getOrientation().getYaw());
            }
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
