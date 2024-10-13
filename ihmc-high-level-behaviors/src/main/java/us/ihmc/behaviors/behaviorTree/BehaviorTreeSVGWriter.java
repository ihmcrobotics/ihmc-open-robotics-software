package us.ihmc.behaviors.behaviorTree;

import org.jfree.svg.SVGGraphics2D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class BehaviorTreeSVGWriter
{
   public BehaviorTreeSVGWriter(BehaviorTreeNodeDefinition node, WorkspaceResourceDirectory saveFileDirectory)
   {
      double documentSize = 1000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);


      svgGraphics2D.drawString(node.getName(), 0, 0);

      Path svgPath = Paths.get("%s.svg".formatted(node.getName()));
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
