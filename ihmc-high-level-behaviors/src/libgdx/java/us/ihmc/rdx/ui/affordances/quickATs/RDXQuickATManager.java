package us.ihmc.rdx.ui.affordances.quickATs;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableFootstep;
import us.ihmc.rdx.ui.affordances.RDXInteractableFootstepPlan;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collection;

public class RDXQuickATManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodySceneNode node;
   private final WorkspaceResourceDirectory resourceDirectory = new WorkspaceResourceDirectory(getClass(), "/quickATs");
   private final ImGuiDirectory quickATDirectory;
   private final ImGuiInputText extraFileNameToSave = new ImGuiInputText("Enter file name (.json)");
   private String loadingFileName = "";
   private boolean fileToSaveHasCompleteName = true;
   private RDXTeleoperationManager teleoperationManager;

   public RDXQuickATManager(RigidBodySceneNode node)
   {
      this.node = node;
      String[] nameParts = node.getName().split(" ");

      quickATDirectory = new ImGuiDirectory(resourceDirectory.getFilesystemDirectory().toString(),
                                       pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                    && Arrays.stream(nameParts)
                                                             .map(String::toLowerCase)
                                                             .anyMatch(pathEntry.path().getFileName().toString().toLowerCase()::contains)
                                                    && pathEntry.path().getFileName().toString().endsWith(".json"),
                                       this::setLoadingFile);
      extraFileNameToSave.setImString(node.getName());

      Collection<RDXPanel> RDXPanels = RDXBaseUI.getInstance().getImGuiPanelManager().getPanels();
      for (RDXPanel panel : RDXPanels)
         if (panel instanceof RDXTeleoperationManager)
         {
            teleoperationManager = (RDXTeleoperationManager) panel;
            break;
         }
   }

   public void renderImGuiWidgets()
   {
      if (!quickATDirectory.isEmpty())
      {
         if (ImGui.button(labels.get("Load AT")))
         {
            load();
         }
         ImGui.sameLine();
         quickATDirectory.renderImGuiWidgetsAsDropDownMenu();
      }

      String saveButtonLabel = fileToSaveHasCompleteName ? "Save AT" : "Confirm Name";
      if (ImGui.button(labels.get(saveButtonLabel)))
      {
         fileToSaveHasCompleteName = !fileToSaveHasCompleteName;
         if (fileToSaveHasCompleteName)
            saveToFile(extraFileNameToSave.getString() + ".json");
      }
      if (!fileToSaveHasCompleteName)
      {
         ImGui.sameLine();
         extraFileNameToSave.render();
         ImGui.pushStyleColor(ImGuiCol.Text, 1.0f, 0.0f, 0.0f, 1.0f);
         ImGui.text("Saving Pending ...");
         ImGui.popStyleColor();
      }
   }

   public void setLoadingFile(String fileName)
   {
      loadingFileName = fileName;
   }

   public void load()
   {
      if (loadingFileName.isEmpty())
         LogTools.warn("Cannot load file - Please select a file from the drop down menu");
      else
      {
         Path filePath = Paths.get(resourceDirectory.getFilesystemDirectory().toString(), loadingFileName);
         if (Files.exists(filePath))
         {
            JSONFileTools.load(filePath, jsonNode ->
            {
               // TODO deal with hand manager as well
               teleoperationManager.getLocomotionManager().getInteractableFootstepPlan().clear();
               RDXManualFootstepPlacement footstepPlacer = teleoperationManager.getLocomotionManager().getManualFootstepPlacement();
               JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode ->
               {
                  // get transform from json
                  RigidBodyTransform soleFrameTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(footstepNode, soleFrameTransform);
                  // create frame pose to express it in world frame
                  FramePose3D soleFrame = new FramePose3D(node.getNodeFrame(), soleFrameTransform);
                  soleFrame.changeFrame(ReferenceFrame.getWorldFrame());
                  footstepPlacer.createNewFootstep(RobotSide.getSideFromString(footstepNode.get("side").asText()));
                  footstepPlacer.setFootstepPose(soleFrame);
                  footstepPlacer.forceFootstepPlacement();
                  footstepPlacer.placeFootstep();
               });
               footstepPlacer.exitPlacement();
            });
            LogTools.info("Loaded file {}", filePath);
         }
         else
         {
            LogTools.warn("Could not load file {}", filePath);
         }
      }
   }

   public void saveToFile(String fileName)
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(resourceDirectory, fileName);
      if (file.isFileAccessAvailable())
      {
         LogTools.info("Saving to file ...");
         JSONFileTools.save(file, jsonNode ->
         {
            jsonNode.put("name", node.getName());
//            jsonNode.put("name", node.getName());
            ArrayNode footstepsArrayNode = jsonNode.putArray("footsteps");
            RDXInteractableFootstepPlan footstepPlan = teleoperationManager.getLocomotionManager().getInteractableFootstepPlan();
            RecyclingArrayList<RDXInteractableFootstep> footsteps = footstepPlan.getFootsteps();
            for (RDXInteractableFootstep footstep : footsteps)
            {
               ObjectNode footstepNode = footstepsArrayNode.addObject();
               footstepNode.put("side", footstep.getPlannedFootstep().getRobotSide().getLowerCaseName());
               // get sole frame pose
               FramePose3D soleFrame = new FramePose3D(footstep.getPlannedFootstep().getFootstepPose());
               soleFrame.changeFrame(node.getNodeFrame());
               // save pose to json
               JSONTools.toJSON(footstepNode, soleFrame);
            }
         });
         LogTools.info("Saved to file {}", file.getFileName());
         quickATDirectory.reindexDirectory();
      }
      else
      {
         LogTools.warn("Could not save to {}", file.getFileName());
      }
   }
}
