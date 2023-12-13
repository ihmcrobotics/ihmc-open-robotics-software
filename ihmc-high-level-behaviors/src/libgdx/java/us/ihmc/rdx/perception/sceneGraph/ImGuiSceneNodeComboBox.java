package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class ImGuiSceneNodeComboBox
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final SceneGraph sceneGraph;
   private final ImInt currentSelectedNodeIndex = new ImInt();
   private SceneNode selectedNode;

   public ImGuiSceneNodeComboBox(SceneGraph sceneGraph)
   {
      this.sceneGraph = sceneGraph;
      selectedNode = sceneGraph.getRootNode();
   }

   public void render()
   {
      String[] sceneNodeNamesArray = sceneGraph.getNodeNameList().toArray(new String[sceneGraph.getNodeNameList().size()]);

      if (ImGui.combo(labels.get("Parent"), currentSelectedNodeIndex, sceneNodeNamesArray))
      {
         selectedNode = sceneGraph.getNamesToNodesMap().get(sceneNodeNamesArray[currentSelectedNodeIndex.get()]);
      }
   }

   public SceneNode getSelectedNode()
   {
      return selectedNode;
   }
}
