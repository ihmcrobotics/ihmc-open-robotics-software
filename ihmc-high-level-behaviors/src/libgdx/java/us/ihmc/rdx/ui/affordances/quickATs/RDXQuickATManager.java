package us.ihmc.rdx.ui.affordances.quickATs;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
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
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   private final ImBoolean saveArms = new ImBoolean(false);
   private final ImBoolean saveFootsteps = new ImBoolean(false);
   private final SideDependentList<ImBoolean> saveArmsSide = new SideDependentList<>();
   private RDXArmControlMode armControlMode = RDXArmControlMode.POSE_WORLD;

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
      {
         if (panel instanceof RDXTeleoperationManager)
         {
            teleoperationManager = (RDXTeleoperationManager) panel;
            break;
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         saveArmsSide.put(side, new ImBoolean(true));
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

      String saveButtonLabel = fileToSaveHasCompleteName ? "Save AT" : "Confirm Options";
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
         ImGui.checkbox(labels.get("Arms"), saveArms);
         if(saveArms.get())
         {
            for (RobotSide side : RobotSide.values)
            {
               ImGui.sameLine();
               ImGui.checkbox(labels.get(side.getLowerCaseName()), saveArmsSide.get(side));
            }
            if (ImGui.radioButton(labels.get("Joint angles"), armControlMode == RDXArmControlMode.JOINT_ANGLES))
            {
               armControlMode = RDXArmControlMode.JOINT_ANGLES;
            }
            if (ImGui.radioButton(labels.get("Hands Pose"), armControlMode == RDXArmControlMode.POSE_WORLD))
            {
               armControlMode = RDXArmControlMode.POSE_WORLD;
            }
         }
         ImGui.checkbox(labels.get("Footsteps"), saveFootsteps);
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
               JSONTools.forEachArrayElement(jsonNode, "arms", armsNode ->
               {
                  RobotSide side = RobotSide.getSideFromString(armsNode.get("side").asText());
                  // get transform from json
                  RigidBodyTransform handFrameTransform = new RigidBodyTransform();
                  JSONTools.toEuclid(armsNode, handFrameTransform);
                  // create frame pose to express it in world frame
                  FramePose3D handFrame = new FramePose3D(node.getNodeFrame(), handFrameTransform);
                  teleoperationManager.getArmManager().setDesiredHandFramePose(side, handFrame);
               });

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

            if (saveArms.get())
            {
               ArrayNode armsArrayNode = jsonNode.putArray("arms");
               RDXArmManager armManager = teleoperationManager.getArmManager();
               for (RobotSide side : RobotSide.values)
               {
                  if (saveArmsSide.get(side).get())
                  {
                     if (armControlMode == RDXArmControlMode.JOINT_ANGLES)
                     {
                        ObjectNode armNode = armsArrayNode.addObject();
                        armNode.put("side", side.getLowerCaseName());
                        double[] jointAngles = armManager.getDesiredJointAngles(side);
                        for (int i = 0; i < jointAngles.length; i++)
                        {
                           armNode.put("j" + i, jointAngles[i]);
                        }
                     }
                     else
                     {
                        FramePose3D desiredHandPose = armManager.getDesiredHandFramePose(side);
                        if (desiredHandPose != null)
                        {
                           ObjectNode armNode = armsArrayNode.addObject();
                           armNode.put("side", side.getLowerCaseName());
                           // get hand pose in node frame
                           desiredHandPose.changeFrame(node.getNodeFrame());
                           // save pose to json
                           JSONTools.toJSON(armNode, desiredHandPose);
                        }
                     }
                  }
               }
            }

            if (saveFootsteps.get())
            {
               ArrayNode footstepsArrayNode = jsonNode.putArray("footsteps");
               RDXInteractableFootstepPlan footstepPlan = teleoperationManager.getLocomotionManager().getInteractableFootstepPlan();
               RecyclingArrayList<RDXInteractableFootstep> footsteps = footstepPlan.getFootsteps();
               for (RDXInteractableFootstep footstep : footsteps)
               {
                  ObjectNode footstepNode = footstepsArrayNode.addObject();
                  footstepNode.put("side", footstep.getPlannedFootstep().getRobotSide().getLowerCaseName());
                  // get sole frame pose
                  FramePose3D solePose = new FramePose3D(footstep.getPlannedFootstep().getFootstepPose());
                  solePose.changeFrame(node.getNodeFrame());
                  // save pose to json
                  JSONTools.toJSON(footstepNode, solePose);
               }
            }
         });
         LogTools.info("Saved to file {}", file.getFileName());
         quickATDirectory.reindexDirectory();
      }
      else
      {
         LogTools.warn("Could not save to {}", file.getFileName());
      }
      resetSavingOptions();
   }

   private void resetSavingOptions()
   {
      for (RobotSide side : RobotSide.values)
      {
         saveArmsSide.get(side).set(true);
      }
      saveArms.set(false);
      saveFootsteps.set(false);
   }
}
