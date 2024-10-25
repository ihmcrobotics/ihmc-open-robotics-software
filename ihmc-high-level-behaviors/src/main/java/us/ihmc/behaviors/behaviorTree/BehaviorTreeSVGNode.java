package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang.WordUtils;
import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;

import java.awt.*;

public class BehaviorTreeSVGNode
{
   private final SVGGraphics2D svgGraphics2D;
   private final BehaviorTreeNodeState<?> node;
   private final Color color;
   private int treeViewX;
   private int treeViewY;
   private int timeViewX;
   private int timeViewY;

   public BehaviorTreeSVGNode(SVGGraphics2D svgGraphics2D,
                              BehaviorTreeNodeState<?> node,
                              BehaviorTreeSVGNode nodeToExecuteAfter,
                              int index,
                              int originX,
                              int originY)
   {
      this.svgGraphics2D = svgGraphics2D;
      this.node = node;

      int nodeDepth = BehaviorTreeTools.getNodeDepth(node);
      int childIndex = BehaviorTreeTools.getChildIndex(node);
      treeViewX = originX + 20 * nodeDepth;
      treeViewY = originY + 30 * index;

      int timeBarHeight = 5;
      timeViewX = originX;
      timeViewY = originY + timeBarHeight * index + 1500;

      if (node instanceof ActionNodeState)
      {
         timeViewX = nodeToExecuteAfter.getTimeViewX();
      }

      color = new Color((int) (Math.random() * 256), (int) (Math.random() * 256), (int) (Math.random() * 256), 100);
//      svgGraphics2D.setColor(color);
//      svgGraphics2D.setStroke(new BasicStroke(0.5f));
//      svgGraphics2D.fillRect(x, y, 200, 30);
//      svgGraphics2D.setColor(Color.GRAY);
//      svgGraphics2D.setStroke(new BasicStroke(0.25f));
//      drawRect(x, y, 12, 10);
//      x += 2;
//      y += 8;
//      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 8));
//      String indexString = "%d".formatted(index);
//      svgGraphics2D.drawString(indexString, x + 4 * (2 - indexString.length()), y);
//      x += 14;
//      y += 4;
      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 12));
      svgGraphics2D.drawString("%s".formatted(filterName(node)), treeViewX, treeViewY);
      treeViewY += 13;
      svgGraphics2D.setColor(Color.GRAY);
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 10));
      svgGraphics2D.drawString("%s".formatted(getTypeName(node.getDefinition())), treeViewX, treeViewY);

      if (nodeDepth > 1)
      {
         int lineX = treeViewX - 15;
         int lineY = treeViewY - 15;
         svgGraphics2D.drawLine(lineX, lineY, lineX, lineY - (childIndex == 0 ? 11 : 30));

         svgGraphics2D.drawLine(lineX, lineY, lineX + 8, lineY);
      }

      svgGraphics2D.setColor(color);
      svgGraphics2D.setStroke(new BasicStroke(0.5f));
      svgGraphics2D.fillRect(timeViewX, timeViewY, 200, timeBarHeight);

//      svgGraphics2D.setColor(color);
//      svgGraphics2D.setStroke(new BasicStroke(0.5f));
//      svgGraphics2D.fillRect(x, y, 200, 30);
//      svgGraphics2D.setColor(Color.GRAY);
//      svgGraphics2D.setStroke(new BasicStroke(0.25f));

   }

   public int getTimeViewX()
   {
      return timeViewX;
   }

   public int getTimeViewY()
   {
      return timeViewY;
   }

   public int getTreeViewX()
   {
      return treeViewX;
   }

   public int getTreeViewY()
   {
      return treeViewY;
   }

   public BehaviorTreeNodeState<?> getNode()
   {
      return node;
   }

   private void drawRect(int x, int y, int width, int height)
   {
      // The supplied drawRect is broken for strokes < 1.0f
      svgGraphics2D.drawLine(x, y, x + width, y);
      svgGraphics2D.drawLine(x, y, x, y + height);
      svgGraphics2D.drawLine(x, y + height, x + width, y + height);
      svgGraphics2D.drawLine(x + width, y, x + width, y + height);
   }

   private String filterName(BehaviorTreeNodeState<?> node)
   {
      String name = node.getDefinition().getName();

      if (name.startsWith("CHECK POINT "))
         name = name.substring("CHECK POINT ".length());
      if (name.startsWith("RASVideo_"))
         name = name.substring("RASVideo_".length());
      if (name.endsWith(".json"))
      {
         name = name.substring(0, name.length() - 5);
         String afterUnderscore = name.substring(name.lastIndexOf("_") + 1);
         String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));
         name = titleCaseString;
      }

      return name;
   }

   private String getTypeName(BehaviorTreeNodeDefinition node)
   {
      if (node.getName().contains("CHECK POINT"))
         return "Checkpoint Node";

      if (node instanceof BehaviorTreeRootNodeDefinition)
         return "Root Node";
      if (node instanceof DoorTraversalDefinition)
         return "Door Traversal Coordinator";
      if (node instanceof ActionSequenceDefinition)
         return "Action Sequence";
      if (node instanceof WaitDurationActionDefinition)
         return "Wait Action";
      if (node instanceof HandPoseActionDefinition)
         return "Hand Pose Action";
      if (node instanceof FootstepPlanActionDefinition)
         return "Walk Action";
      if (node instanceof ChestOrientationActionDefinition)
         return "Chest Trajectory Action";
      if (node instanceof SakeHandCommandActionDefinition)
         return "Finger Trajectory Action";
      if (node instanceof ScrewPrimitiveActionDefinition)
         return "Screw Trajectory Action";
      return "";
   }
}
