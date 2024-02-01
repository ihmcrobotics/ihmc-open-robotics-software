package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImVec2;
import imgui.extension.imnodes.ImNodes;
import org.abego.treelayout.Configuration;
import org.abego.treelayout.NodeExtentProvider;
import org.abego.treelayout.TreeLayout;
import org.abego.treelayout.util.DefaultTreeForTreeLayout;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;

import java.awt.geom.Rectangle2D;
import java.util.HashMap;
import java.util.Map;

/**
 * This was a work in progress to see if we could automate the layout of
 * the behavior tree UI panels. It turned out to be tricky and maybe not
 * the greatest direction to go in.
 *
 * Update 10/12/2023 Actually using this tree layout may be exactly what we
 * need when going to smaller icons and shapes for the nodes. - @dcalvert
 *
 * @deprecated Needs work.
 */
public class ImNodesAbegoAutoLayout
{
   private final HashMap<Integer, ImVec2> nodeSizeMap = new HashMap<>();
   private int nodeIndex = 0;

   public void layoutNodes(RDXBehaviorUIInterface tree)
   {
      DefaultTreeForTreeLayout<RDXBehaviorUIInterface> treeForLayout = new DefaultTreeForTreeLayout<>(tree);
      constructAbegoTree(tree, treeForLayout);

      TreeLayout<RDXBehaviorUIInterface> layout = new TreeLayout<RDXBehaviorUIInterface>(treeForLayout, new NodeExtentProvider<RDXBehaviorUIInterface>()
      {

         @Override
         public double getWidth(RDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSizeMap.get(index).x;
         }

         @Override
         public double getHeight(RDXBehaviorUIInterface gdxBehaviorUIInterface)
         {
            int index = getIndexOfNode(gdxBehaviorUIInterface, tree);
            return nodeSizeMap.get(index).y;
         }
      }, new Configuration<RDXBehaviorUIInterface>()
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
         public double getGapBetweenNodes(RDXBehaviorUIInterface node1, RDXBehaviorUIInterface node2)
         {
            return 25;
         }
      });

      Map<RDXBehaviorUIInterface, Rectangle2D.Double> map = layout.getNodeBounds();
      for (RDXBehaviorUIInterface node : map.keySet())
      {
         int index = getIndexOfNode(node, tree);
         Rectangle2D.Double pos = map.get(node);

         ImNodes.setNodeGridSpacePos(index, (float) pos.x, (float) pos.y);
      }
   }


   private int getIndexOfNode(RDXBehaviorUIInterface node, RDXBehaviorUIInterface tree)
   {
      nodeIndex = tree.generateUID();
      return getIndexOfNodeInternal(node, tree);
   }

   private int getIndexOfNodeInternal(RDXBehaviorUIInterface node, RDXBehaviorUIInterface root)
   {
      if (root.equals(node))
         return nodeIndex;
      else
      {
         for (RDXBehaviorUIInterface child : root.getUIChildren())
         {
            nodeIndex++;
            int val = getIndexOfNodeInternal(node, child);

            if (val != -1)
               return val;
         }
      }

      return -1;
   }

   private void constructAbegoTree(RDXBehaviorUIInterface tree, DefaultTreeForTreeLayout<RDXBehaviorUIInterface> layout)
   {
      if (tree == null)
         return;

      for (RDXBehaviorUIInterface child : tree.getUIChildren())
      {
         layout.addChild(tree, child);
         constructAbegoTree(child, layout);
      }
   }
}
