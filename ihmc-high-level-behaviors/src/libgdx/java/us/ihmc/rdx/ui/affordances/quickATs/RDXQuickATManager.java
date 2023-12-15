package us.ihmc.rdx.ui.affordances.quickATs;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.imgui.ImGuiInputText;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
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
import java.util.*;

public class RDXQuickATManager extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private SceneGraph sceneGraph;
   private SceneNode defaultNode = new SceneNode(100, "");
   private SceneNode selectedNode = defaultNode;
   private final WorkspaceResourceDirectory resourceDirectory = new WorkspaceResourceDirectory(getClass(), "/quickATs");
   private ImGuiDirectory quickATDirectory;
   private final ImGuiInputText extraFileNameToSave = new ImGuiInputText("Enter file name (.json)");
   private String loadingFileName = "";
   private boolean fileToSaveHasCompleteName = true;
   private RDXTeleoperationManager teleoperationManager;
   private final ImBoolean saveArms = new ImBoolean(false);
   private final ImBoolean saveFootsteps = new ImBoolean(false);
   private final SideDependentList<ImBoolean> saveArmsSide = new SideDependentList<>();
   private RDXArmControlMode armControlMode = RDXArmControlMode.POSE_WORLD;

   private boolean ATTracking = false;
   private SideDependentList<RigidBodyTransform> handFrameTransforms = new SideDependentList<>();
   private final SideDependentList<Double> timeLastCommandHand = new SideDependentList<>();

   private List<String> nodeNames;
   private transient String[] selectableNodeNameArray = new String[0];
   private int selectedIndex = 0;

   public RDXQuickATManager()
   {
      super("Quick AT Manager");
      setRenderMethod(this::renderImGuiWidgets);

      for (RobotSide side : RobotSide.values)
      {
         saveArmsSide.put(side, new ImBoolean(true));
         timeLastCommandHand.put(side, 0.0);
      }
   }

   public void create(RDXTeleoperationManager teleoperationManager, SceneGraph sceneGraph)
   {
      this.teleoperationManager = teleoperationManager;
      this.sceneGraph = sceneGraph;
      nodeNames = sceneGraph.getNodeNameList();

      quickATDirectory = new ImGuiDirectory(resourceDirectory.getFilesystemDirectory().toString(),
                                            pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                         && pathEntry.path().getFileName().toString().endsWith(".json"),
                                            this::setLoadingFile);
   }

   public void update()
   {
      nodeNames = sceneGraph.getNodeNameList();

      if (ATTracking)
      {
         boolean freezeTracking = false;
         if (selectedNode instanceof CenterposeNode centerposeNode)
         {
            if (centerposeNode.getConfidence() < 0.3 || !centerposeNode.getCurrentlyDetected())
            {
               freezeTracking = true;
            }
         }

         //TODO add other interactable link poses
         if (!freezeTracking)
         {
            for (RobotSide side : handFrameTransforms.keySet())
            {
               // create frame pose to express it in world frame
               FramePose3D handFrame = new FramePose3D(selectedNode.getNodeFrame(), handFrameTransforms.get(side));
               teleoperationManager.getArmManager().setDesiredHandFramePose(side, handFrame);
               teleoperationManager.getArmManager().computeTrajectoryTime(side, 0.5, 5.0);

               double timeInTrajectory = System.nanoTime() / 1_000_000_000.0 - timeLastCommandHand.get(side);
               timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, teleoperationManager.getArmManager().getTrajectoryTime(side));
               double alpha = timeInTrajectory / teleoperationManager.getArmManager().getTrajectoryTime(side);

               double[] qInitials = teleoperationManager.getArmManager().getCurrentJointAngles(side);
               double[] qGoals = teleoperationManager.getArmManager().getDesiredJointAngles(side);
               double[] qDesireds = new double[qInitials.length];
               double[] qDDesireds = new double[qInitials.length];

               for (int i = 0; i < qInitials.length; i++)
               {
                  double qDes = EuclidCoreTools.interpolate(qInitials[i], qGoals[i], alpha);
                  double qDDes;
                  if (alpha <= 0.0 || alpha >= 1.0)
                     qDDes = 0.0;
                  else
                     qDDes = (qGoals[i] - qInitials[i]) / teleoperationManager.getArmManager().getTrajectoryTime(side);
                  qDesireds[i] = qDes;
                  qDDesireds[i] = qDDes;
               }

               timeLastCommandHand.replace(side, System.nanoTime() / 1_000_000_000.0);
               teleoperationManager.getArmManager().moveHand(side, qDesireds, qDDesireds);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (nodeNames.size() > 0)
      {
         selectableNodeNameArray = nodeNames.toArray(selectableNodeNameArray);
         if (ImGui.beginCombo(labels.get("Select node"), selectableNodeNameArray[selectedIndex]))
         {
            for (int i = 0; i < selectableNodeNameArray.length; i++)
            {
               String pathName = selectableNodeNameArray[i];
               if (ImGui.selectable(pathName, selectedIndex == i))
               {
                  selectedIndex = i;
                  selectedNode = sceneGraph.getNamesToNodesMap().get(selectableNodeNameArray[selectedIndex]);
                  extraFileNameToSave.setImString(selectedNode.getName());
               }
            }
            ImGui.endCombo();
         }
      }

      if (!quickATDirectory.isEmpty() && selectedIndex != 0)
      {
         if (ImGui.button(labels.get("Load AT")))
         {
            load();
         }
         ImGui.sameLine();
         quickATDirectory.renderImGuiWidgetsAsDropDownMenu();
      }

      if (!selectedNode.equals(defaultNode))
      {
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
            if (saveArms.get())
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

      if (!loadingFileName.isEmpty())
      {
         boolean changedColorPreviewButton = false;
         if (ATTracking)
         {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
            changedColorPreviewButton = true;
         }
         if (ImGui.button(labels.get("AT Tracking")))
         {
            ATTracking = !ATTracking;
         }
         if (changedColorPreviewButton)
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
                  handFrameTransforms.put(side, handFrameTransform);
                  // create frame pose to express it in world frame
                  FramePose3D handFrame = new FramePose3D(selectedNode.getNodeFrame(), handFrameTransform);
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
                  FramePose3D soleFrame = new FramePose3D(selectedNode.getNodeFrame(), soleFrameTransform);
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
            jsonNode.put("name", selectedNode.getName());

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
                           desiredHandPose.changeFrame(selectedNode.getNodeFrame());
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
                  solePose.changeFrame(selectedNode.getNodeFrame());
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
