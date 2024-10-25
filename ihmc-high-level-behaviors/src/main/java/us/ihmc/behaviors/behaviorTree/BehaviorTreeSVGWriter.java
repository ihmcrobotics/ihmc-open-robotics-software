package us.ihmc.behaviors.behaviorTree;

import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.log.LogTools;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class BehaviorTreeSVGWriter
{
   private int i = 0;
   private int x = 100;
   private int y = 100;
   private BehaviorTreeSVGNode actionSequenceSVGNode;
//   private int actionSequenceIndex = 0;
//   private int actionSequenceX = x;
//   private int actionSequenceY = y;
   private final ArrayList<BehaviorTreeSVGNode> svgNodes = new ArrayList<>();

   public BehaviorTreeSVGWriter(BehaviorTreeNodeState node)
   {
      double documentSize = 2000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);


      BehaviorTreeTools.runForSubtreeNodes(node, child ->
      {
//         if (child instanceof ActionSequenceState actionSequenceState)
//         {
//            actionSequenceIndex = i;
//            actionSequenceX = x;
//            actionSequenceY = y;
//         }

//         String[] firstNodesInSection = { "Set static for approach",
//                                          "Set static for grasp",
//                                          "Pull door screw primitive",
//                                          "Left arm against panel"};
//         String[] sectionNames = { "Approach",
//                                   "Turn handle",
//                                   "Open door",
//                                   "Walk through"};
//
//         boolean isFirstNodeInSection = false;
//         int j = 0;
//         for (; j < firstNodesInSection.length; j++)
//         {
//            isFirstNodeInSection = child.getDefinition().getName().equals(firstNodesInSection[j]);
//            if (isFirstNodeInSection)
//               break;
//         }

//         if ((svgNodes.size() - (actionSequenceIndex + 1)) % 12 == 0)
//         if (isFirstNodeInSection)
//            y = actionSequenceY + 30;
//
//         if (isFirstNodeInSection)
//         {
//            x += 16;
//            y += 12;
//            svgGraphics2D.setColor(Color.BLACK);
//            svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 12));
//            svgGraphics2D.drawString(sectionNames[j - 1], x, y);
//            y += 18;
//            x -= 16;
//         }

         BehaviorTreeSVGNode nodeToExecuteAfter = null;
         if (child instanceof ActionNodeState actionNode)
         {
            if (actionNode.getDefinition() instanceof ActionNodeDefinition actionNodeDefinition)
            {
               if (actionNodeDefinition.getExecuteAfterPrevious().getValue())
               {
                  nodeToExecuteAfter = svgNodes.get(svgNodes.size() - 1);
               }
               else if (actionNodeDefinition.getExecuteAfterBeginning().getValue())
               {
                  nodeToExecuteAfter = actionSequenceSVGNode;
               }
               else
               {
                  long afterID = actionNodeDefinition.getExecuteAfterNodeID().getValue();
                  for (BehaviorTreeSVGNode otherNode : svgNodes)
                  {
                     if (otherNode.getNode() instanceof ActionNodeState existingActionNode)
                     {
                        if (existingActionNode.getID() == afterID)
                        {
                           nodeToExecuteAfter = otherNode;
                        }
                     }
                  }
               }
            }
         }

         BehaviorTreeSVGNode svgNode = new BehaviorTreeSVGNode(svgGraphics2D, child, nodeToExecuteAfter, actionSequenceSVGNode, i, x, y);
         svgNodes.add(svgNode);

         if (child instanceof ActionSequenceState)
            actionSequenceSVGNode = svgNode;

         x += svgNode.getWidth();
         y += svgNode.getHeight();

         ++i;
      });

      Path svgPath = Paths.get("%s.svg".formatted(node.getDefinition().getName()));
      LogTools.info("Saving SVG to {}", svgPath);

      try (FileWriter writer = new FileWriter(svgPath.toFile()))
      {
         String svgDocument = svgGraphics2D.getSVGDocument();
         // Add viewBox attribute to the SVG element, to make it load correctly in Inkscape
         svgDocument = svgDocument.replace("<svg", "<svg viewBox=\"0 0 %f %f\"".formatted(documentSize, documentSize));
         writer.write(svgDocument);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
