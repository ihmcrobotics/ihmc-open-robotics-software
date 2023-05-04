package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXInteractableBox;
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
   private RigidBodyTransform handTransformToWorld = new RigidBodyTransform();
   private RigidBodyTransform objectToWorld = new RigidBodyTransform();
   private final PoseReferenceFrame affordanceFrame = new PoseReferenceFrame("affordanceFrame", ReferenceFrame.getWorldFrame());
   private RDXInteractableObjectBuilder objectBuilder;

   private RDXInteractableBox object;


   private RDXInteractableBox dummyHand;
   private RDXInteractableDummyHand interactableHand;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   // this is expressed in interacting object's frame
   private FramePose3D graspPose = new FramePose3D();
   private ImBoolean showGraspPose = new ImBoolean(true);
   private RDXReferenceFrameGraphic graspPoseGraphic;

   // this is expressed in interacting object's frame
   private FramePose3D pressPose = new FramePose3D();
   private ImBoolean showPressPose = new ImBoolean(true);
   private RDXReferenceFrameGraphic pressPoseGraphic;


   private final PoseReferenceFrame graspFrame = new PoseReferenceFrame("graspFrame", affordanceFrame);
   private final PoseReferenceFrame pressFrame = new PoseReferenceFrame("pressFrame", affordanceFrame);
   private ArrayList<FramePose3D> customPoses = new ArrayList<>();
   private ArrayList<PoseReferenceFrame> customFrames = new ArrayList<>();
   private ArrayList<RDXReferenceFrameGraphic> customPoseGraphics = new ArrayList<>();

   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/boxAffordance");
   private boolean initialized = false;

   private final ArrayList<String> affordanceNames = new ArrayList<>();
   private final ImString customAffordanceName = new ImString();
   private final ArrayList<Color> colors = new ArrayList<>(Arrays.asList(Color.CORAL, Color.BLUE, Color.OLIVE, Color.BROWN,
                                                                         Color.ORANGE, Color.BLUE, Color.MAGENTA, Color.FOREST));

   private final ImGuiInputText textInput = new ImGuiInputText("Add custom affordance");
   private int colorIndex = 0;

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


            // create the manager for the desired arm setpoints
            Box3D box3D = new Box3D();
            FramePose3D boxPose = new FramePose3D();
            Vector3D dimensions = new Vector3D(1, 1, 1);
            box3D.set(boxPose, dimensions);

            object = new RDXInteractableBox(baseUI, new Box3D(0.1,0.1,0.1),"box");

            Box3D dummyBox = new Box3D();
            FramePose3D dummyPose = new FramePose3D();
            dummyPose.appendTranslation(0.0, 1.0, 0.0);
            Vector3D dummyDim = new Vector3D(0.5, 1.0, 0.5);
            dummyBox.set(dummyPose, dummyDim);

            dummyHand = new RDXInteractableBox(baseUI, dummyBox, "dummyHand");
            dummyHand.setColor(Color.WHITE);
            baseUI.getPrimaryScene().addRenderableProvider(dummyHand);

            interactableHand = new RDXInteractableDummyHand(baseUI.getPrimary3DPanel(),
                                                            handTransformToWorld);

            graspPoseGraphic = new RDXReferenceFrameGraphic(0.3, Color.WHITE);
            pressPoseGraphic = new RDXReferenceFrameGraphic(0.3, Color.BLACK);

            baseUI.getPrimaryScene().addRenderableProvider(interactableHand);
            baseUI.getPrimaryScene().addRenderableProvider(objectBuilder.getSelectedObject());
            baseUI.getPrimaryScene().addRenderableProvider(RDXAffordanceEditorUI.this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Affordance Develop Panel", RDXAffordanceEditorUI.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            dummyHand.update();
            object.update();
            updateFramePosesWRTBox();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose() {
            baseUI.dispose();
         }
      });
   }

   public void updateFramePosesWRTBox()
   {
      FramePose3D boxPose = new FramePose3D(object.getPose3DGizmo().getPose());
      affordanceFrame.setPoseAndUpdate(boxPose);
      affordanceFrame.update();
      // grab
      graspPose = new FramePose3D(graspFrame);
      graspPose.changeFrame(ReferenceFrame.getWorldFrame());
      graspPoseGraphic.updateFromFramePose(graspPose);
      // press
      pressPose = new FramePose3D(pressFrame);
      pressPose.changeFrame(ReferenceFrame.getWorldFrame());
      pressPoseGraphic.updateFromFramePose(pressPose);

      for (int i = 0; i < customPoses.size(); ++i)
      {
         customPoses.set(i, new FramePose3D(customFrames.get(i)));
         customPoses.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         customPoseGraphics.get(i).updateFromFramePose(customPoses.get(i));
      }
   }

   public void renderImGuiWidgets()
   {
      if (!initialized)
      {
         loadAllFromJSON("box");
         initialized = true;

         FramePose3D boxPose = new FramePose3D(object.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         affordanceFrame.setPoseAndUpdate(boxPose);

         graspPose.changeFrame(affordanceFrame);
         graspFrame.setPoseAndUpdate(graspPose);

         pressPose.changeFrame(affordanceFrame);
         pressFrame.setPoseAndUpdate(pressPose);
      }

      if (ImGui.button(labels.get("Record grasp point w.r.t box"))) {
         FramePose3D boxPose = new FramePose3D(object.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         affordanceFrame.setPoseAndUpdate(boxPose);

         FramePose3D handPoseInBoxFrame = new FramePose3D(dummyHand.getPose3DGizmo().getPose());
         handPoseInBoxFrame.changeFrame(affordanceFrame);
         graspFrame.setPoseAndUpdate(handPoseInBoxFrame);
      }

      ImGui.checkbox(labels.get("grasping pose"), showGraspPose);

      ImGui.separator();

      if (ImGui.button("Record press point w.r.t box")) {
         FramePose3D boxPose = new FramePose3D(object.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         affordanceFrame.setPoseAndUpdate(boxPose);

         FramePose3D handPoseInBoxFrame = new FramePose3D(dummyHand.getPose3DGizmo().getPose());
         handPoseInBoxFrame.changeFrame(affordanceFrame);
         pressFrame.setPoseAndUpdate(handPoseInBoxFrame);
      }
      ImGui.checkbox(labels.get("press pose"), showPressPose);

      ImGui.separator();

      ImGui.text("Default affordances");
      if (ImGui.button(labels.get("Grasp Point")))
      {
         // read from json
         loadFromJSON(object.getPose3DGizmo().getPose(), "box", "pre-graspingPoint");
         // move hand to grasp point
         dummyHand.getPose3DGizmo().getTransformToParent().set(graspPose);
      }

      if (ImGui.button(labels.get("Press Point")))
      {
         // read from json
         loadFromJSON(object.getPose3DGizmo().getPose(), "box", "pressPoint");
         // move hand to grasp point
         dummyHand.getPose3DGizmo().getTransformToParent().set(pressPose);
      }

      if (affordanceNames.size() > 0)
      {
         ImGui.text("Registered custom affordances");
         for (int i = 0; i < affordanceNames.size(); ++i) {
            if (ImGui.button(labels.get(affordanceNames.get(i)))) {
               // move hand to custom point
               FramePose3D customPose = customPoses.get(i);
               dummyHand.getPose3DGizmo().getTransformToParent().set(customPose);
            }
         }
      }

      // custom affordance save input
      if (textInput.render())
      {
         customAffordanceName.set(textInput.getString());

         // check if same name has been used, if so, overwrite
         boolean sameName = false;
         int index = 0;
         for (int i = 0; i < affordanceNames.size(); ++i)
         {
            if (Objects.equals(affordanceNames.get(i), customAffordanceName.get()))
            {
               sameName = true;
               index = i;
               break;
            }
         }

         FramePose3D boxPose = new FramePose3D(object.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         affordanceFrame.setPoseAndUpdate(boxPose);
         FramePose3D handPoseInBoxFrame = new FramePose3D(dummyHand.getPose3DGizmo().getPose());
         handPoseInBoxFrame.changeFrame(affordanceFrame);
         PoseReferenceFrame frame = new PoseReferenceFrame(customAffordanceName.get() +"Frame", affordanceFrame);
         frame.setPoseAndUpdate(handPoseInBoxFrame);

         if (sameName)
         {
            customPoses.set(index, handPoseInBoxFrame);
            customFrames.set(index, frame);
         }
         else
         {
            affordanceNames.add(customAffordanceName.get());
            customPoses.add(handPoseInBoxFrame);
            customFrames.add(frame);
            customPoseGraphics.add(new RDXReferenceFrameGraphic(0.3, colors.get(colorIndex % colors.size())));
            colorIndex++;
         }
      }

      if (ImGui.button("Save to json"))
      {
         saveToJSON();
      }

      ImGui.separator();

      if (ImGui.button(labels.get("Load all affordance points from json")))
      {
         loadAllFromJSON("box");
      }

      if (ImGui.button(labels.get("Clear custom affordances"))) {
         affordanceNames.clear();
         customFrames.clear();
         customPoses.clear();
         customPoseGraphics.clear();
         colorIndex = 0;
         saveToJSON();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraspPose.get())
      {
         graspPoseGraphic.getRenderables(renderables, pool);
      }
      if (showPressPose.get())
      {
         pressPoseGraphic.getRenderables(renderables, pool);
      }

      if (object != null)
         object.getRenderables(renderables, pool);

      for (RDXReferenceFrameGraphic graphic : customPoseGraphics)
      {
         graphic.getRenderables(renderables, pool);
      }
   }

   public void loadAllFromJSON(String objectName)
   {
      WorkspaceFile file = new WorkspaceFile(configurationsDirectory, objectName + "Affordance.json");
      Path filePath = file.getFilesystemFile();
      JSONFileTools.load(filePath, jsonNode ->
      {
         customPoses.clear();
         customFrames.clear();
         affordanceNames.clear();
         customPoseGraphics.clear();
         colorIndex = 0;
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
               graspPose = new FramePose3D(affordanceFrame);
               graspPose.set(x, y, z, yaw, pitch, roll);
               graspPose.changeFrame(ReferenceFrame.getWorldFrame());
               graspPoseGraphic.updateFromFramePose(graspPose);
            }
            else if (affordanceName.contains("press"))
            {
               pressPose = new FramePose3D(affordanceFrame);
               pressPose.set(x, y, z, yaw, pitch, roll);
               pressPose.changeFrame(ReferenceFrame.getWorldFrame());
               pressPoseGraphic.updateFromFramePose(pressPose);
            }
            else
            {
               affordanceNames.add(affordanceName);
               FramePose3D pose = new FramePose3D(affordanceFrame);
               pose.set(x, y, z, yaw, pitch, roll);
               pose.changeFrame(ReferenceFrame.getWorldFrame());
               customPoses.add(pose);
               customPoseGraphics.add(new RDXReferenceFrameGraphic(0.3, colors.get(colorIndex % colors.size())));
               colorIndex++;
               PoseReferenceFrame frame = new PoseReferenceFrame(customAffordanceName.get() +"Frame", affordanceFrame);
               pose.changeFrame(affordanceFrame);
               frame.setPoseAndUpdate(pose);
               customFrames.add(frame);
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
            graspPose = new FramePose3D(affordanceFrame);
            graspPose.set(x, y, z, yaw, pitch, roll);
            graspPose.changeFrame(ReferenceFrame.getWorldFrame());
            graspPoseGraphic.updateFromFramePose(graspPose);
         }

         else if (affordanceName.contains("press"))
         {
            pressPose = new FramePose3D(affordanceFrame);
            pressPose.set(x, y, z, yaw, pitch, roll);
            pressPose.changeFrame(ReferenceFrame.getWorldFrame());
            pressPoseGraphic.updateFromFramePose(pressPose);
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
            ObjectNode node = root.putObject("pre-graspingPoint");

            FramePose3D handPose = new FramePose3D(graspFrame);
            handPose.changeFrame(affordanceFrame);

            node.put("referenceFrame", handPose.getReferenceFrame().toString());
            node.put("x", handPose.getPosition().getX());
            node.put("y", handPose.getPosition().getY());
            node.put("z", handPose.getPosition().getZ());
            node.put("roll", handPose.getOrientation().getRoll());
            node.put("pitch", handPose.getOrientation().getPitch());
            node.put("yaw", handPose.getOrientation().getYaw());

            // affordance 2
            ObjectNode pressNode = root.putObject("pressPoint");

            handPose = new FramePose3D(pressFrame);
            handPose.changeFrame(affordanceFrame);

            pressNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            pressNode.put("x", handPose.getPosition().getX());
            pressNode.put("y", handPose.getPosition().getY());
            pressNode.put("z", handPose.getPosition().getZ());
            pressNode.put("roll", handPose.getOrientation().getRoll());
            pressNode.put("pitch", handPose.getOrientation().getPitch());
            pressNode.put("yaw", handPose.getOrientation().getYaw());

            for (int i = 0; i < affordanceNames.size(); ++i)
            {
               ObjectNode customNode = root.putObject(affordanceNames.get(i));
               handPose = new FramePose3D(customFrames.get(i));
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
