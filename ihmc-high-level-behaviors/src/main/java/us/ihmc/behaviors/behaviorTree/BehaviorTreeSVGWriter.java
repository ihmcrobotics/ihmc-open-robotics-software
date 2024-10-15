package us.ihmc.behaviors.behaviorTree;

import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.log.LogTools;

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
   private int actionSequenceIndex = 0;
   private int actionSequenceX = x;
   private int actionSequenceY = y;
   private final ArrayList<BehaviorTreeSVGNode> svgNodes = new ArrayList<>();

   public BehaviorTreeSVGWriter(BehaviorTreeNodeState node)
   {
      double documentSize = 2000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);


      BehaviorTreeTools.runForSubtreeNodes(node, child ->
      {
         if (child instanceof ActionSequenceState)
         {
            actionSequenceIndex = i;
            actionSequenceX = x;
            actionSequenceY = y;
         }

         if (child.getDefinition().getName().equals("Set static for approach"))
         {

         }


         BehaviorTreeSVGNode svgNode = new BehaviorTreeSVGNode(svgGraphics2D, child, svgNodes, i, x, y);
         svgNodes.add(svgNode);

         y += svgNode.getHeight();

         if ((svgNodes.size() - (actionSequenceIndex + 1)) % 12 == 0)
            y = actionSequenceY + 30;

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
