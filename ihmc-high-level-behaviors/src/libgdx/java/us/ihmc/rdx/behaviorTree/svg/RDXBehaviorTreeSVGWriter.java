package us.ihmc.rdx.behaviorTree.svg;

import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Helps draw the tree to an SVG format for formal presentation such
 * as in scientific publications.
 */
public class RDXBehaviorTreeSVGWriter
{
   private int i = 0;
   private final Map<RDXBehaviorTreeNode<?, ?>, RDXBehaviorTreeSVGNode> map = new HashMap<>();
   private final ArrayList<RDXBehaviorTreeSVGNode> svgNodes = new ArrayList<>();

   public RDXBehaviorTreeSVGWriter(RDXBehaviorTreeNode<?, ?> node)
   {
      double documentSize = 2000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);

      BehaviorTreeTools.runForSubtreeNodes(node, child ->
      {
         RDXBehaviorTreeSVGNode svgNode = new RDXBehaviorTreeSVGNode(svgGraphics2D, child, i, map);
         map.put(child, svgNode);
         svgNodes.add(svgNode);

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
