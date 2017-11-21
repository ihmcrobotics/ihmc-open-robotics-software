package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;
import us.ihmc.plotting.Plotter;

public class SpatialNodePlotter
{
   private int dimensionOfConfigurations;
   private double trajectoryTime;

   private List<Plotter> plotters = new ArrayList<Plotter>();
   private double[] upperLimits;
   private double[] lowerLimits;

   private int cnt;

   public SpatialNodePlotter(WholeBodyTrajectoryToolboxData toolboxData, boolean enabled)
   {
      this.dimensionOfConfigurations = toolboxData.getExplorationDimension();

      this.upperLimits = new double[dimensionOfConfigurations];
      this.lowerLimits = new double[dimensionOfConfigurations];
      this.trajectoryTime = toolboxData.getTrajectoryTime();

      for (int i = 0; i < dimensionOfConfigurations; i++)
      {
         Plotter plotter = new Plotter();

         plotter.setPreferredSize(700, 700);

         plotter.setViewRange(1.5);
         plotter.setXYZoomEnabled(true);
         plotter.setShowLabels(true);
         plotter.setFocusPointX(0.5);
         plotter.setFocusPointY(0.5);
         plotters.add(plotter);
         upperLimits[i] = Double.NEGATIVE_INFINITY;
         lowerLimits[i] = Double.POSITIVE_INFINITY;

         String plotterName = toolboxData.createRandomNode().getConfigurationName(i);

         if (enabled)
            plotter.showInNewWindow(plotterName, false);
      }
      cnt = 0;
   }

   /**
    * 1::node
    * 2::path
    * 3::shortcut
    */
   public void update(SpatialNode node, int type)
   {
      double normalizedTime = node.getTime() / trajectoryTime;
      Color color;
      for (int nodeIndex = 0; nodeIndex < dimensionOfConfigurations; nodeIndex++)
      {
         String prefix;
         switch (type)
         {
         case 1:
            if (node.isValid())
            {
               prefix = "" + cnt + "_valid_" + nodeIndex;
               color = Color.blue;
            }
            else
            {
               prefix = "" + cnt + "_invalid_" + nodeIndex;
               color = Color.red;
            }
            break;
         case 2:
            prefix = "" + cnt + "_path_" + nodeIndex;
            color = Color.black;
            break;
         case 3:
            prefix = "" + cnt + "_shortcut_" + nodeIndex;
            color = Color.green;
            break;
         default:
            prefix = "";
            color = Color.white;
            break;
         }

         double configurationData = node.getConfigurationData(nodeIndex);

         if (node.getParent() != null && node.isValid())
         {
            SpatialNode parentNode = node.getParent();
            double parentTime = parentNode.getTime() / trajectoryTime;
            double parentConfigurationData = parentNode.getConfigurationData(nodeIndex);

            LineArtifact lineArtifact = new LineArtifact(prefix + "_line", new Point2D(parentTime, parentConfigurationData),
                                                         new Point2D(normalizedTime, configurationData));

            lineArtifact.setColor(color);
            plotters.get(nodeIndex).addArtifact(lineArtifact);
         }

         CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_node", normalizedTime, configurationData, 0.02, true);
         nodeArtifact.setColor(color);

         plotters.get(nodeIndex).addArtifact(nodeArtifact);
         plotters.get(nodeIndex).update();
      }

      cnt++;
   }
}
