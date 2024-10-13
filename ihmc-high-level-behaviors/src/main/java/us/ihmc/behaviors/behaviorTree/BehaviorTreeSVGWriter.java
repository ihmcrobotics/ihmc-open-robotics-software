package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang.WordUtils;
import org.jfree.svg.SVGGraphics2D;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class BehaviorTreeSVGWriter
{
   private int x = 100;
   private int y = 100;

   public BehaviorTreeSVGWriter(BehaviorTreeNodeDefinition node, WorkspaceResourceDirectory saveFileDirectory)
   {
      double documentSize = 1000.0;
      SVGGraphics2D svgGraphics2D = new SVGGraphics2D(documentSize, documentSize);

//      svgGraphics2D.drawString(node.getName(), x, y);

      BehaviorTreeTools.runForSubtreeNodes(node, child ->
      {
         y += 15;
         svgGraphics2D.setColor(Color.BLACK);
         svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 12));
         svgGraphics2D.drawString("%s".formatted(filterName(child)), x, y);
         y += 13;
         svgGraphics2D.setColor(Color.GRAY);
         svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 10));
         svgGraphics2D.drawString("%s".formatted(getTypeName(child)), x, y);
         y += 3;
      });

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

   private String filterName(BehaviorTreeNodeDefinition node)
   {
      String name = node.getName();

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
