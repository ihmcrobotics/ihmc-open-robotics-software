package us.ihmc.rdx.ui.teleoperation;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFoot;
import us.ihmc.rdx.ui.affordances.RDXInteractableHand;
import us.ihmc.rdx.ui.affordances.RDXInteractableRobotLink;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.*;

public class RDXTeleoperationScriptedPoses extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString saveFileName = new ImString();
   private final WorkspaceResourceDirectory posesDirectory = new WorkspaceResourceDirectory(getClass(), "/poses");
   private final ImGuiDirectory imPosesDirectory;
   private final SideDependentList<RDXInteractableHand> interactableHands;
   private final SideDependentList<RDXInteractableFoot> interactableFeet;
   private final RDXInteractableRobotLink interactableChest;
   private final RDXInteractableRobotLink interactablePelvis;
   private final Runnable clearInteractables;

   public RDXTeleoperationScriptedPoses(SideDependentList<RDXInteractableHand> interactableHands,
                                        SideDependentList<RDXInteractableFoot> interactableFeet,
                                        RDXInteractableRobotLink interactableChest,
                                        RDXInteractableRobotLink interactablePelvis,
                                        Runnable clearInteractables)
   {
      super("Scripted Poses");
      this.interactableHands = interactableHands;
      this.interactableFeet = interactableFeet;
      this.interactableChest = interactableChest;
      this.interactablePelvis = interactablePelvis;
      this.clearInteractables = clearInteractables;

      setRenderMethod(this::renderImGuiWidgets);

      imPosesDirectory = new ImGuiDirectory(posesDirectory.getFilesystemDirectory().toString(),
                                            name -> true,
                                            pathEntry -> true,
                                            this::pathSelected);
   }

   private void renderImGuiWidgets()
   {
      imPosesDirectory.renderImGuiWidgets();

      ImGui.text("Save current interactable poses as:");
      ImGuiTools.inputText(labels.getHidden("saveFileName"), saveFileName);
      ImGui.sameLine();
      ImGui.text(".json");
      if (!saveFileName.get().isEmpty() && !saveFileName.get().endsWith(".json"))
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Save Pose")))
         {
            JSONFileTools.save(new WorkspaceResourceFile(posesDirectory, saveFileName.get() + ".json"), jsonRootObjectNode ->
            {
               ArrayNode interactablesArrayNode = jsonRootObjectNode.putArray("interactables");

               for (RobotSide side : RobotSide.values)
               {
                  interactableToJSON("Hand", side, interactablesArrayNode, interactableHands.get(side));
                  interactableToJSON("Foot", side, interactablesArrayNode, interactableFeet.get(side));
               }

               interactableToJSON("Chest", null, interactablesArrayNode, interactableChest);
               interactableToJSON("Pelvis", null, interactablesArrayNode, interactablePelvis);
            });
         }
      }
   }

   private void interactableToJSON(String type, RobotSide side, ArrayNode interactablesArrayNode, RDXInteractableRobotLink interactableRobotLink)
   {
      if (interactableRobotLink.isModified())
      {
         ObjectNode handNode = interactablesArrayNode.addObject();
         handNode.put("type", type);
         if (side != null)
            handNode.put("side", side.getLowerCaseName());
         JSONTools.toJSON(handNode, interactableRobotLink.getPose());
      }
   }

   private void pathSelected(String pathName)
   {
      WorkspaceResourceFile fileToLoad = new WorkspaceResourceFile(posesDirectory, pathName);
      LogTools.info("Loading {}", fileToLoad.getFilesystemFile().toString());

      clearInteractables.run();

      JSONFileTools.load(fileToLoad, jsonRootObjectNode ->
      {
         JSONTools.forEachArrayElement(jsonRootObjectNode, "interactables", objectNode ->
         {
            RDXInteractableRobotLink interactableRobotLink = null;

            if (objectNode.get("type").asText().equals("Hand"))
            {
               RobotSide side = objectNode.get("side").asText().equals(RobotSide.LEFT.getLowerCaseName()) ? RobotSide.LEFT : RobotSide.RIGHT;
               interactableRobotLink = interactableHands.get(side);
            }
            else if (objectNode.get("type").asText().equals("Foot"))
            {
               RobotSide side = objectNode.get("side").asText().equals(RobotSide.LEFT.getLowerCaseName()) ? RobotSide.LEFT : RobotSide.RIGHT;
               interactableRobotLink = interactableFeet.get(side);
            }
            else if (objectNode.get("type").asText().equals("Chest"))
            {
               interactableRobotLink = interactableChest;
            }
            else if (objectNode.get("type").asText().equals("Pelvis"))
            {
               interactableRobotLink = interactablePelvis;
            }

            if (interactableRobotLink != null)
            {
               JSONTools.toEuclid(objectNode, interactableRobotLink.getPose3DGizmo().getTransformToParent());
               interactableRobotLink.setSelected();
            }
         });
      });
   }
}
