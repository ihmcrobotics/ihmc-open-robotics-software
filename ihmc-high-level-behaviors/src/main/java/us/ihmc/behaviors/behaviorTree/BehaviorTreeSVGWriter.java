package us.ihmc.behaviors.behaviorTree;

import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceState;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

/**
 * Helps draw the tree to an SVG format for formal presentation such
 * as in scientific publications.
 */
public class BehaviorTreeSVGWriter
{
   private int i = 0;
   private int x = 100;
   private int y = 100;
   private BehaviorTreeSVGNode actionSequenceSVGNode;
   private final ArrayList<BehaviorTreeSVGNode> svgNodes = new ArrayList<>();

   public BehaviorTreeSVGWriter(BehaviorTreeNodeState node)
   {
      double documentSize = 2000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);

      BehaviorTreeTools.runForSubtreeNodes(node, child ->
      {
         BehaviorTreeSVGNode nodeToExecuteAfter = null;
         if (child instanceof ActionNodeState actionNode)
         {
            if (actionNode.getDefinition() instanceof ActionNodeDefinition actionNodeDefinition)
            {
               if (actionNodeDefinition.getExecuteAfterPrevious())
               {
                  nodeToExecuteAfter = svgNodes.get(svgNodes.size() - 1);
               }
               else if (actionNodeDefinition.getExecuteAfterBeginning())
               {
                  nodeToExecuteAfter = actionSequenceSVGNode;
               }
               else
               {
                  for (BehaviorTreeSVGNode otherNode : svgNodes)
                  {
                     if (otherNode.getNode() instanceof ActionNodeState existingActionNode)
                     {
                        if (existingActionNode.getID() == actionNodeDefinition.getExecuteAfterNodeID())
                        {
                           nodeToExecuteAfter = otherNode;
                        }
                     }
                  }
               }
            }
         }

//         String lowerCaseName = child.getDefinition().getName().toLowerCase();
//         if (lowerCaseName.contains("set static") || lowerCaseName.contains("check point"))
//            return;

         BehaviorTreeSVGNode svgNode = new BehaviorTreeSVGNode(svgGraphics2D, child, nodeToExecuteAfter, i, x, y);
         svgNodes.add(svgNode);

         if (child instanceof ActionSequenceState)
            actionSequenceSVGNode = svgNode;

         ++i;
      });

      Path svgPath = Paths.get("%s.svg".formatted(node.getDefinition().getName()));
      LogTools.info("Saving SVG to {}", svgPath);

      FileTools.ensureFileExists(svgPath, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

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
