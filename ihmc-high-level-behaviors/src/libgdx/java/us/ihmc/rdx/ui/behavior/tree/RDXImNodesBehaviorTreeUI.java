package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImColor;
import imgui.extension.imnodes.ImNodes;
import imgui.extension.imnodes.flag.ImNodesColorStyle;
import imgui.extension.imnodes.flag.ImNodesMiniMapLocation;
import imgui.extension.imnodes.flag.ImNodesStyleVar;
import imgui.internal.ImGui;
import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.math3.util.Pair;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImNodesTools;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;

public class RDXImNodesBehaviorTreeUI
{
   private RDXImNodesTreeNode rootNode;
   private final ArrayList<RDXImNodesTreeNode> allNodesList = new ArrayList<>();
   private int linkIndex = 0;
   private boolean firstRun = true;
   private final WorkspaceResourceDirectory configurationsDirectory = new WorkspaceResourceDirectory(getClass(), "/imnodeTrees");
   private final MutableInt pinIndex = new MutableInt(0);

   public void create()
   {
      ImNodesTools.initialize();

      float backgroundColor;
      float gridColor;
      if (Boolean.parseBoolean(System.getProperty("imgui.dark"))) // TODO: Tune dark colors
      {
         backgroundColor = 0.95f;
         gridColor = 0.9f;
      }
      else
      {
         backgroundColor = 0.95f;
         gridColor = 0.9f;
      }

      ImNodes.pushColorStyle(ImNodesColorStyle.GridBackground, ImColor.floatToColor(gridColor, gridColor, gridColor));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackground, ImColor.floatToColor(backgroundColor, backgroundColor, backgroundColor));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackgroundHovered, ImColor.floatToColor(backgroundColor, backgroundColor, backgroundColor));
      ImNodes.pushColorStyle(ImNodesColorStyle.NodeBackgroundSelected, ImColor.floatToColor(backgroundColor, backgroundColor, backgroundColor));
      ImNodes.pushColorStyle(ImNodesColorStyle.Link, ImColor.floatToColor(0.0f, 0.0f, 0.0f));
      ImNodes.pushColorStyle(ImNodesColorStyle.LinkHovered, ImColor.floatToColor(0.0f, 0.0f, 0.0f));
      ImNodes.pushColorStyle(ImNodesColorStyle.LinkSelected, ImColor.floatToColor(0.0f, 0.0f, 0.0f));
      ImNodes.pushStyleVar(ImNodesStyleVar.NodeBorderThickness, 5.0f);
      ImNodes.pushStyleVar(ImNodesStyleVar.PinCircleRadius, 0.0f);
   }

   public void setRootNode(RDXBehaviorUIInterface rootBehaviorUI)
   {
      allNodesList.clear();
      rootNode = new RDXImNodesTreeNode(rootBehaviorUI, rootBehaviorUI.generateUID(), pinIndex);
      addNodesToList(rootNode);
   }

   private void addNodesToList(RDXImNodesTreeNode node)
   {
      allNodesList.add(node);
      for (RDXImNodesTreeNode child : node.getChildren())
      {
         addNodesToList(child);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushFont(ImGuiTools.getSmallFont());
      ImNodes.beginNodeEditor();
      ArrayList<Pair<Integer, Integer>> links = new ArrayList<>();
      renderNodeAndChildren(rootNode, -1, links);
      renderLinks(links);

      if (firstRun)
      {
         firstRun = false;
         loadLayoutNodesFromFile();
      }

      ImNodes.miniMap(0.1f, ImNodesMiniMapLocation.BottomRight);
      ImNodes.endNodeEditor();
      ImGui.popFont();
   }

   private void renderNodeAndChildren(RDXImNodesTreeNode node, int parentPinIndex, ArrayList<Pair<Integer, Integer>> links)
   {
      node.render(parentPinIndex, links);

      for (RDXImNodesTreeNode child : node.getChildren())
      {
         renderNodeAndChildren(child, parentPinIndex++, links);
      }
   }

   private void renderLinks(ArrayList<Pair<Integer, Integer>> links)
   {
      for (Pair<Integer, Integer> p : links)
      {
         ImNodes.link(linkIndex++, p.getFirst(), p.getSecond());
      }
   }

   private void loadLayoutNodesFromFile()
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory,
                                                             rootNode.getBehaviorNodeUI().getUIChildren().get(0).getDefinition().getName());
      LogTools.info("Loading imnodes layout from {}", file.getPathForResourceLoadingPathFiltered());
      JSONFileTools.load(file.getClasspathResourceAsStream(), jsonNode ->
      {
         JsonNode treeNodesNode = jsonNode.get("treeNodes");
         Iterator<Map.Entry<String, JsonNode>> it = treeNodesNode.fields();
         while (it.hasNext())
         {
            Map.Entry<String, JsonNode> entry = it.next();

            for (RDXImNodesTreeNode node : allNodesList)
            {
               if (node.getBehaviorNodeUI().getDefinition().getName().equals(entry.getKey()))
               {
                  String[] pos = entry.getValue().asText().split(",");
                  float x = Float.parseFloat(pos[0]);
                  float y = Float.parseFloat(pos[1]);

                  ImNodes.setNodeGridSpacePos(node.getNodeID(), x, y);
               }
            }
         }
      });
   }

   public void saveLayoutToFile()
   {
      WorkspaceResourceFile file = new WorkspaceResourceFile(configurationsDirectory,
                                                             rootNode.getBehaviorNodeUI().getUIChildren().get(0).getDefinition().getName());
      if (file.isFileAccessAvailable())
      {
         LogTools.info("Saving imnodes layout to {}", WorkspacePathTools.removePathPartsBeforeProjectFolder(file.getFilesystemFile()));
         JSONFileTools.save(file, root ->
         {
            ObjectNode treeNodesNode = root.putObject("treeNodes");
            for (RDXImNodesTreeNode node : allNodesList)
            {
               treeNodesNode.put(node.getBehaviorNodeUI().getDefinition().getName(),
                                 ImNodes.getNodeGridSpacePosX(node.getNodeID()) + "," + ImNodes.getNodeGridSpacePosY(node.getNodeID()));

            }
         });
      }
   }

   public void destroy()
   {
      ImNodesTools.destroy();
   }
}