package us.ihmc.gdx.ui.behavior.tree;

import imgui.ImVec2;
import imgui.extension.imnodes.ImNodes;
import org.abego.treelayout.Configuration;
import org.abego.treelayout.NodeExtentProvider;
import org.abego.treelayout.TreeLayout;
import org.abego.treelayout.util.DefaultTreeForTreeLayout;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;

import java.awt.geom.Rectangle2D;
import java.util.HashMap;
import java.util.Map;

public class ImNodesAbegoAutoLayout
{
   private final HashMap<Integer, ImVec2> nodeSizeMap = new HashMap<>();
   private int nodeIndex = 0;

   public void layoutNodes(ImGuiGDXBehaviorUIInterface tree)
   {
      DefaultTreeForTreeLayout<ImGuiGDXBehaviorUIInterface> treeForLayout = new DefaultTreeForTreeLayout<>(tree);
      constructAbegoTree(tree, treeForLayout);

      TreeLayout<ImGuiGDXBehaviorUIInterface> layout = new TreeLayout<ImGuiGDXBehaviorUIInterface>(treeForLayout, new NodeExtentProvider<ImGuiGDXBehaviorUIInterface>()
      {

         @Override
         public double getWidth(ImGuiGDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSizeMap.get(index).x;
         }

         @Override
         public double getHeight(ImGuiGDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSizeMap.get(index).y;
         }
      }, new Configuration<ImGuiGDXBehaviorUIInterface>()
      {
         @Override
         public Location getRootLocation()
         {
            return Location.Top;
         }

         @Override
         public AlignmentInLevel getAlignmentInLevel()
         {
            return AlignmentInLevel.AwayFromRoot;
         }

         @Override
         public double getGapBetweenLevels(int nextLevel)
         {
            return 50;
         }

         @Override
         public double getGapBetweenNodes(ImGuiGDXBehaviorUIInterface node1, ImGuiGDXBehaviorUIInterface node2)
         {
            return 25;
         }
      });

      Map<ImGuiGDXBehaviorUIInterface, Rectangle2D.Double> map = layout.getNodeBounds();
      for (ImGuiGDXBehaviorUIInterface node : map.keySet())
      {
         int index = getIndexOfNode(node, tree);
         Rectangle2D.Double pos = map.get(node);

         ImNodes.setNodeGridSpacePos(index, (float) pos.x, (float) pos.y);
      }
   }


   private int getIndexOfNode(ImGuiGDXBehaviorUIInterface node, ImGuiGDXBehaviorUIInterface tree)
   {
      nodeIndex = tree.generateUID();
      return getIndexOfNodeInternal(node, tree);
   }

   private int getIndexOfNodeInternal(ImGuiGDXBehaviorUIInterface node, ImGuiGDXBehaviorUIInterface root)
   {
      if (root.equals(node))
         return nodeIndex;
      else
      {
         for (ImGuiGDXBehaviorUIInterface child : root.getUIChildren())
         {
            nodeIndex++;
            int val = getIndexOfNodeInternal(node, child);

            if (val != -1)
               return val;
         }
      }

      return -1;
   }

   private void constructAbegoTree(ImGuiGDXBehaviorUIInterface tree, DefaultTreeForTreeLayout<ImGuiGDXBehaviorUIInterface> layout)
   {
      if (tree == null)
         return;

      for (ImGuiGDXBehaviorUIInterface child : tree.getUIChildren())
      {
         layout.addChild(tree, child);
         constructAbegoTree(child, layout);
      }
   }
}
