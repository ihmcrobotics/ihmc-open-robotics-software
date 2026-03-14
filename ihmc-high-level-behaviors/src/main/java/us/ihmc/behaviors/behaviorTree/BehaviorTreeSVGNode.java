package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.text.WordUtils;
import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SpineActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WalkActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ArmActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.SakeHandCommandActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.ScrewPrimitiveActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionState;

import java.awt.*;
import java.util.Random;

/**
 * Helps draw the tree node to an SVG format for formal presentation such
 * as in scientific publications.
 */
public class BehaviorTreeSVGNode
{
   private static final Random random = new Random(0L);

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

      color = new Color((int) (random.nextDouble() * 256), (int) (random.nextDouble() * 256), (int) (random.nextDouble() * 256), 100);

      int nodeDepth = BehaviorTreeTools.getNodeDepth(node);
      int childIndex = BehaviorTreeTools.getChildIndex(node);
      treeViewX = originX + 20 * nodeDepth;
      treeViewY = originY + 30 * index;

      int timeBarHeight = 5;
      timeViewX = treeViewX;
      timeViewY = originY + timeBarHeight * index + 1100;

      if (node instanceof ActionNodeState)
      {
         timeViewX = nodeToExecuteAfter.getTimeViewX();
      }


      svgGraphics2D.setColor(color);
      svgGraphics2D.setStroke(new BasicStroke(0.5f));

      boolean drawBoxes = false;
      int colorBoxSize = 15;
      if (drawBoxes)
         svgGraphics2D.fillRect(treeViewX, treeViewY - 7, colorBoxSize, colorBoxSize);
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
      int colorBoxSpaceX = drawBoxes ? colorBoxSize + 5 : 0;
      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 12));
      svgGraphics2D.drawString("%s".formatted(filterName(node)), treeViewX + colorBoxSpaceX, treeViewY);
      treeViewY += 13;
      svgGraphics2D.setColor(Color.GRAY);
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 10));
      svgGraphics2D.drawString("%s".formatted(getTypeName(node.getDefinition())), treeViewX + colorBoxSpaceX, treeViewY);

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

      double secondsToPixels = 15.0;
      double duration = 0.0f;
      if (node instanceof WaitDurationActionState action)
      {
         duration = action.getDefinition().getWaitDuration();
      }
      else if (node instanceof ArmActionState action)
      {
         duration = action.getDefinition().getTrajectoryDuration();
      }
      else if (node instanceof SpineActionState action)
      {
         duration = action.getDefinition().getTrajectoryDuration();
      }
      else if (node instanceof WalkActionState action)
      {
         duration = 10.0; // TODO
      }
      else if (node instanceof SakeHandCommandActionState action)
      {
         duration = 0.5; // TODO
      }
      else if (node instanceof ScrewPrimitiveActionState action)
      {
         duration = 2.0; // TODO
      }


      timeViewX += (int) Math.round(duration * secondsToPixels);

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
      if (node instanceof ArmActionDefinition)
         return "Arm Action";
      if (node instanceof WalkActionDefinition)
         return "Walk Action";
      if (node instanceof SpineActionDefinition)
         return "Spine Action";
      if (node instanceof SakeHandCommandActionDefinition)
         return "Finger Trajectory Action";
      if (node instanceof ScrewPrimitiveActionDefinition)
         return "Screw Trajectory Action";
      return "";
   }
}
