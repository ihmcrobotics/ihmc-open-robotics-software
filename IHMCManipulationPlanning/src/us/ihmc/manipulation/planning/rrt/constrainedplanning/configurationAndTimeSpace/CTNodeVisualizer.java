package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.awt.Color;

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
      if(enabled)
         plotter.showInNewWindow(ctaskName, false);
   }

   public void updateVisualizer(CTTaskNode newNode)
   {
      String prefix = ""+configurationIndex+""+updateCnt;
      if (newNode.getIsValidNode())
      {
         CTTaskNode parentNode = newNode.getParentNode();
         LineArtifact nodeArtifact = new LineArtifact(prefix, new Point2D(parentNode.getNormalizedNodeData(0), parentNode.getNormalizedNodeData(configurationIndex)),
                                                      new Point2D(newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex)));
         nodeArtifact.setColor(Color.blue);
         plotter.addArtifact(nodeArtifact);
         
         CircleArtifact nodeArtifact1 = new CircleArtifact(prefix, newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex), 0.01, true);
         nodeArtifact1.setColor(Color.blue);
         CircleArtifact nodeArtifact2 = new CircleArtifact(prefix, parentNode.getNormalizedNodeData(0), parentNode.getNormalizedNodeData(configurationIndex), 0.01, true);
         nodeArtifact2.setColor(Color.blue);
         plotter.addArtifact(nodeArtifact1);
         plotter.addArtifact(nodeArtifact2);
      }
      else
      {
         CircleArtifact nodeArtifact = new CircleArtifact(prefix, newNode.getNormalizedNodeData(0), newNode.getNormalizedNodeData(configurationIndex), 0.01, true);
         nodeArtifact.setColor(Color.red);
         plotter.addArtifact(nodeArtifact);
      }
      plotter.update();
      updateCnt++;
   }
}
