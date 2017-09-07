package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class CTNodeVisualizer
{
   private Plotter plotter;

   private int configurationIndex;

   private int updateCnt;

   private boolean enabled;
   private String ctaskName;

   public CTNodeVisualizer(String ctaskName, int configurationIndex, boolean enabled)
   {
      this.ctaskName = ctaskName;
      this.plotter = new Plotter();

      this.configurationIndex = configurationIndex;

      /*
       * zoom- always normalized.
       */
      plotter.setPreferredSize(700, 700);

      plotter.setViewRange(1.5);
      plotter.setXYZoomEnabled(true);
      plotter.setShowLabels(true);
      plotter.setFocusPointX(0.5);
      plotter.setFocusPointY(0.5);

      this.enabled = enabled;
   }

   public void initialize()
   {
      updateCnt = 0;
      if (enabled)
         plotter.showInNewWindow(ctaskName, false);
   }

   public void updateVisualizer(CTTaskNode newNode)
   {
      String prefix = "" + configurationIndex + "" + updateCnt;
      if (newNode.getValidity())
      {
         if (newNode.getParentNode() != null)
         {
            CTTaskNode parentNode = newNode.getParentNode();
            LineArtifact lineArtifact = new LineArtifact(prefix + "_line",
                                                         new Point2D(parentNode.getNormalizedNodeData(0), parentNode.getNormalizedNodeData(configurationIndex)),
                                                         new Point2D(newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex)));

            lineArtifact.setColor(Color.blue);
            plotter.addArtifact(lineArtifact);
         }

         CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_valid", newNode.getNormalizedNodeData(0),
                                                          newNode.getNormalizedNodeData(configurationIndex), 0.01, true);
         nodeArtifact.setColor(Color.blue);

         plotter.addArtifact(nodeArtifact);

      }
      else
      {
         CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_invalid", newNode.getNormalizedNodeData(0),
                                                          newNode.getNormalizedNodeData(configurationIndex), 0.01, true);
         nodeArtifact.setColor(Color.red);
         plotter.addArtifact(nodeArtifact);
      }
      plotter.update();
      updateCnt++;
   }

   private void updateVisualizer(CTTaskNode newNode, boolean isPath)
   {
      String prefix = "" + configurationIndex + "" + updateCnt;

      if (newNode.getParentNode() != null)
      {
         CTTaskNode parentNode = newNode.getParentNode();
         LineArtifact lineArtifact = new LineArtifact(prefix + "_line",
                                                      new Point2D(parentNode.getNormalizedNodeData(0), parentNode.getNormalizedNodeData(configurationIndex)),
                                                      new Point2D(newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex)));

         lineArtifact.setColor(Color.green);
         plotter.addArtifact(lineArtifact);
      }

      CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_valid", newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex),
                                                       0.015, true);
      nodeArtifact.setColor(Color.green);

      plotter.addArtifact(nodeArtifact);

      plotter.update();
      updateCnt++;
   }

   public void updateVisualizer(ArrayList<CTTaskNode> path)
   {
      int size = path.size();
      for(int i=0;i<size;i++)
         updateVisualizer(path.get(i), true);
   }
}
