package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.awt.Color;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class CTNodeVisualizer
{
   private Plotter plotter;

   private int configurationIndex;

   private int updateCnt;

   public CTNodeVisualizer(int configurationIndex)
   {
      this.plotter = new Plotter();

      this.configurationIndex = configurationIndex;

      /*
       * zoom- always normalized.
       */
      plotter.setPreferredSize(800, 600);

      plotter.setViewRange(10.0);
      plotter.setXYZoomEnabled(true);
      plotter.setShowLabels(true);      
   }
   
   public void initialize()
   {
      updateCnt = 0;
      plotter.showInNewWindow();
   }

   public void updateVisualizer(CTTaskNode newNode)
   {
      String prefix = ""+configurationIndex+""+updateCnt;
      PrintTools.info(prefix);
      if (newNode.getIsValidNode())
      {
         CTTaskNode parentNode = newNode.getParentNode();
         LineArtifact nodeArtifact = new LineArtifact(prefix, new Point2D(parentNode.getNormalizedNodeData(0), parentNode.getNormalizedNodeData(configurationIndex)),
                                                      new Point2D(newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex)));
         nodeArtifact.setColor(Color.blue);
         plotter.addArtifact(nodeArtifact);
         
         CircleArtifact nodeArtifact1 = new CircleArtifact(prefix, newNode.getNormalizedNodeData(0), newNode.getNodeData(configurationIndex), 0.01, true);
         nodeArtifact1.setColor(Color.blue);
         CircleArtifact nodeArtifact2 = new CircleArtifact(prefix, parentNode.getNormalizedNodeData(0), parentNode.getNodeData(configurationIndex), 0.01, true);
         nodeArtifact2.setColor(Color.blue);
         plotter.addArtifact(nodeArtifact1);
         plotter.addArtifact(nodeArtifact2);
      }
      else
      {
         CircleArtifact nodeArtifact = new CircleArtifact(prefix, newNode.getNormalizedNodeData(0), newNode.getNodeData(configurationIndex), 0.01, true);
         nodeArtifact.setColor(Color.red);
         plotter.addArtifact(nodeArtifact);
      }
      plotter.update();
      updateCnt++;
   }
}
