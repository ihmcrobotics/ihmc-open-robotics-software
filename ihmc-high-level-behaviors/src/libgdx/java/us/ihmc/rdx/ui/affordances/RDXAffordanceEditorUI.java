package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private final FramePose3D handPose = new FramePose3D(ReferenceFrame.getWorldFrame(),handTransformToWorld);
   private RDXInteractableObjectBuilder objectBuilder;
   private RigidBodyTransform objectToWorld = new RigidBodyTransform();
   private final PoseReferenceFrame affordanceFrame = new PoseReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   // initial and grasp pose
   private FramePose3D initialPose = new FramePose3D();
   private RDXReferenceFrameGraphic initialPoseGraphic;
   private final PoseReferenceFrame initialFrame = new PoseReferenceFrame("initialFrame", affordanceFrame);
   private FramePose3D graspPose = new FramePose3D();
   private RDXReferenceFrameGraphic graspPoseGraphic;
   private final PoseReferenceFrame graspFrame = new PoseReferenceFrame("graspFrame", affordanceFrame);
   // pre-grasp poses
   private ArrayList<FramePose3D> preGraspPoses = new ArrayList<>();
   private ArrayList<PoseReferenceFrame> preGraspFrames = new ArrayList<>();
   private ArrayList<RDXReferenceFrameGraphic> preGraspPoseGraphics = new ArrayList<>();
   private final ArrayList<Color> preGraspColors = new ArrayList<>(Arrays.asList(new Color(0xFFE4B5FF),
                                                                                 new Color(0xFF8C00FF),
                                                                                 new Color(0xFFDAB9FF),
                                                                                 new Color(0xFF6600FF),
                                                                                 new Color(0xFFA07AFF)));
   // post-grasp poses
   private ArrayList<FramePose3D> postGraspPoses = new ArrayList<>();
   private ArrayList<PoseReferenceFrame> postGraspFrames = new ArrayList<>();
   private ArrayList<RDXReferenceFrameGraphic> postGraspPoseGraphics = new ArrayList<>();
   private final ArrayList<Color> postGraspColors = new ArrayList<>(Arrays.asList(new Color(0xD8BFD8FF),
                                                                                  new Color(0xBA55D3FF),
                                                                                  new Color(0x9932CCFF),
                                                                                  new Color(0x8A2BE2FF),
                                                                                  new Color(0x4B0082FF)));

   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/boxAffordance");

   private final ArrayList<Integer> preGraspIndices = new ArrayList<>();
   private int preGraspIndex = 0;
   private int preGraspColorIndex = 0;

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

            interactableHand = new RDXInteractableDummyHand(baseUI.getPrimary3DPanel(), handTransformToWorld);
            baseUI.getPrimaryScene().addRenderableProvider(interactableHand);
            baseUI.getPrimaryScene().addRenderableProvider(objectBuilder.getSelectedObject());
            baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceEditorUI.this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Affordance Panel", RDXAffordanceEditorUI.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            updateFramePoses();
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

   public void updateFramePoses()
   {
      if (objectBuilder.isAnyObjectSelected())
      {
         FramePose3D objectPose = new FramePose3D(objectBuilder.getSelectedObject().getPose3DGizmo().getPose());
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
   }

   public void renderImGuiWidgets()
   {
      if ((ImGui.button(labels.get("Set initial pre-grasp frame")) || (ImGui.isKeyReleased(ImGuiTools.getSpaceKey()) && true)))
      {
         initialFrame.setPoseAndUpdate(handPose);
      }

      if (ImGui.button(labels.get("Add pre-grasp frame")))
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

      if (ImGui.button("Set grasp frame"))
      {
         graspFrame.setPoseAndUpdate(handPose);
      }

      if (ImGui.button(labels.get("Initial pre-grasp frame")))
         handTransformToWorld.set(initialPose);  // move hand to pregrasp point

      if (preGraspIndices.size() > 0)
      {
         ImGui.text("Registered pre-grasp frames");
         for (int i = 0; i < preGraspIndices.size(); ++i)
         {
            if (ImGui.button(labels.get(preGraspIndices.get(i).toString())))
            {
               // move hand to selected frame
               FramePose3D selectedPose = preGraspPoses.get(i);
               handTransformToWorld.set(selectedPose);
            }
         }
      }

      if (ImGui.button(labels.get("Grasp frame")))
         handTransformToWorld.set(graspPose); // move hand to grasp point

      if (ImGui.button("Save to json"))
      {
         saveToJSON();
      }

      ImGui.separator();

      if (ImGui.button(labels.get("Load all affordance points from json")))
      {
         loadAllFromJSON("box");
      }

      if (ImGui.button(labels.get("Clear custom affordances")))
      {
         preGraspIndices.clear();
         preGraspFrames.clear();
         preGraspPoses.clear();
         preGraspPoseGraphics.clear();
         preGraspColorIndex = 0;
         saveToJSON();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      initialPoseGraphic.getRenderables(renderables, pool);
      graspPoseGraphic.getRenderables(renderables, pool);
      for (RDXReferenceFrameGraphic customPoseGraphic : preGraspPoseGraphics)
      {
         customPoseGraphic.getRenderables(renderables, pool);
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
