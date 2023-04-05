package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.nio.file.Path;

public class RDXBoxAffordanceDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXInteractableBox interactableBox;
   private RDXInteractableBox dummyHand;

   private ImGuiPanel affordancePanel = new ImGuiPanel("Affordance developer");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   // this is expressed in interacting object's frame
   private FramePose3D graspPose = new FramePose3D();
   private ImBoolean showGraspPose = new ImBoolean(true);
   private RDXReferenceFrameGraphic graspPoseGraphic;

   private FramePose3D pressPose = new FramePose3D();
   private ImBoolean showPressPose = new ImBoolean(true);
   private RDXReferenceFrameGraphic pressPoseGraphic;

   private final PoseReferenceFrame boxFrame = new PoseReferenceFrame("boxFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame graspFrame = new PoseReferenceFrame("graspFrame", boxFrame);
   private final PoseReferenceFrame pressFrame = new PoseReferenceFrame("pressFrame", boxFrame);
   private final FramePose3D tempPose = new FramePose3D();

   private RDXAxisBody axisBody;

   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/boxAffordance");

   private boolean initialized = false;

   public RDXBoxAffordanceDemo() {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter() {
         @Override
         public void create() {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            // create the manager for the desired arm setpoints
            Box3D box3D = new Box3D();
            FramePose3D boxPose = new FramePose3D();
            Vector3D dimensions = new Vector3D(1, 1, 1);
            box3D.set(boxPose, dimensions);

            interactableBox = new RDXInteractableBox(baseUI, box3D, "box");
            interactableBox.setColor(Color.ORANGE);
            baseUI.getPrimaryScene().addRenderableProvider(interactableBox);

            Box3D dummyBox = new Box3D();
            FramePose3D dummyPose = new FramePose3D();
            dummyPose.appendTranslation(0.0, 1.0, 0.0);
            Vector3D dummyDim = new Vector3D(0.5, 1.0, 0.5);
            dummyBox.set(dummyPose, dummyDim);

            dummyHand = new RDXInteractableBox(baseUI, dummyBox, "dummyHand");
            dummyHand.setColor(Color.WHITE);
            baseUI.getPrimaryScene().addRenderableProvider(dummyHand);

            graspPoseGraphic = new RDXReferenceFrameGraphic(0.3, Color.WHITE);
            pressPoseGraphic = new RDXReferenceFrameGraphic(0.3, Color.BLACK);

            axisBody = new RDXAxisBody(baseUI);
            axisBody.getPoseGizmo().getTransformToParent().appendTranslation(0.0, -1.0, 0.0);
            axisBody.update();
            baseUI.getPrimaryScene().addRenderableProvider(axisBody);

//            baseUI.getPrimaryScene().addRenderableProvider(graspPoseGraphic);
            baseUI.getPrimaryScene().addRenderableProvider(RDXBoxAffordanceDemo.this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Affordance Develop Panel", RDXBoxAffordanceDemo.this::renderImGuiWidgets);
         }

         @Override
         public void render() {
            // 1st: let's draw line at point (x1, y1, z1) , (x2, y2, z2)
            dummyHand.update();
            interactableBox.update();
            axisBody.update();
//            update();
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
      FramePose3D boxPose = new FramePose3D(interactableBox.getPose3DGizmo().getPose());
      boxFrame.setPoseAndUpdate(boxPose);
      boxFrame.update();
//      if (update)
//      {
         graspPose = new FramePose3D(graspFrame);
         graspPose.changeFrame(ReferenceFrame.getWorldFrame());
         graspPoseGraphic.updateFromFramePose(graspPose);

         pressPose = new FramePose3D(pressFrame);
         pressPose.changeFrame(ReferenceFrame.getWorldFrame());
         pressPoseGraphic.updateFromFramePose(pressPose);
//      }
   }

   private boolean update = false;

   public void renderImGuiWidgets()
   {
      if (!initialized)
      {
         loadFromJSON(interactableBox.getPose3DGizmo().getPose(), "box", "pre-graspingPoint");
         loadFromJSON(interactableBox.getPose3DGizmo().getPose(), "box", "pressPoint");
         initialized = true;

         FramePose3D boxPose = new FramePose3D(interactableBox.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         boxFrame.setPoseAndUpdate(boxPose);

         graspPose.changeFrame(boxFrame);
         graspFrame.setPoseAndUpdate(graspPose);

         pressPose.changeFrame(boxFrame);
         pressFrame.setPoseAndUpdate(pressPose);
      }

      if (ImGui.button(labels.get("record hand grasp pose w.r.t box")))
      {
         FramePose3D boxPose = new FramePose3D(interactableBox.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         boxFrame.setPoseAndUpdate(boxPose);

         FramePose3D handPoseInBoxFrame = new FramePose3D(dummyHand.getPose3DGizmo().getPose());
         handPoseInBoxFrame.changeFrame(boxFrame);
         graspFrame.setPoseAndUpdate(handPoseInBoxFrame);

         update = true;
      }

      ImGui.checkbox(labels.get("grasping pose"), showGraspPose);

      ImGui.separator();

      if (ImGui.button("record hand push pose w.r.t box"))
      {
         FramePose3D boxPose = new FramePose3D(interactableBox.getPose3DGizmo().getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         boxFrame.setPoseAndUpdate(boxPose);

         FramePose3D handPoseInBoxFrame = new FramePose3D(dummyHand.getPose3DGizmo().getPose());
         handPoseInBoxFrame.changeFrame(boxFrame);
         pressFrame.setPoseAndUpdate(handPoseInBoxFrame);
         update = true;
      }
      ImGui.checkbox(labels.get("press pose"), showPressPose);

      ImGui.separator();

      if (ImGui.button("SAVE To JSON"))
      {
         saveToJSON();
      }

      ImGui.separator();

      if (ImGui.button(labels.get("Move hand to grasp point from affordance template JSON")))
      {
         // read from json
         loadFromJSON(interactableBox.getPose3DGizmo().getPose(), "box", "pre-graspingPoint");
         // move hand to grasp point
         dummyHand.getPose3DGizmo().getTransformToParent().set(graspPose);
      }

      if (ImGui.button(labels.get("Move hand to press")))
      {
         // read from json
         loadFromJSON(interactableBox.getPose3DGizmo().getPose(), "box", "pressPoint");
         // move hand to grasp point
         dummyHand.getPose3DGizmo().getTransformToParent().set(pressPose);
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

         FramePose3D poseToAppend = new FramePose3D(boxFrame);
         poseToAppend.set(x, y, z, yaw, pitch, roll);

         if (affordanceName.contains("grasp"))
         {
            graspPose = new FramePose3D(boxFrame);
            graspPose.set(x, y, z, yaw, pitch, roll);
            graspPose.changeFrame(ReferenceFrame.getWorldFrame());
            graspPoseGraphic.updateFromFramePose(graspPose);
         }

         else if (affordanceName.contains("press"))
         {
            pressPose = new FramePose3D(boxFrame);
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
            handPose.changeFrame(boxFrame);

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
            handPose.changeFrame(boxFrame);

            pressNode.put("referenceFrame", handPose.getReferenceFrame().toString());
            pressNode.put("x", handPose.getPosition().getX());
            pressNode.put("y", handPose.getPosition().getY());
            pressNode.put("z", handPose.getPosition().getZ());
            pressNode.put("roll", handPose.getOrientation().getRoll());
            pressNode.put("pitch", handPose.getOrientation().getPitch());
            pressNode.put("yaw", handPose.getOrientation().getYaw());
         });
         LogTools.info("SAVED affordance to json");
      }
      else
      {
         LogTools.warn("Could not write to " + file.toString());
      }
   }

   public static void main(String[] args)
   {
      new RDXBoxAffordanceDemo();
   }
}
