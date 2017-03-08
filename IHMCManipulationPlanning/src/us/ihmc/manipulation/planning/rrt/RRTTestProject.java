package us.ihmc.manipulation.planning.rrt;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Scanner;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.tools.io.printing.PrintTools;

class DrawPanel extends JPanel
{
   double xUpper;
   double xLower;

   double yUpper;
   double yLower;

   double scale;

   int sizeU;
   int sizeV;

   double[] allPoints;
   int numOfPoints;

   double[] pathPoints;
   double[] boxInfo = new double[4];

   RRTPlanner info;

   DrawPanel(int scaleF, double[] pointInfo, RRTPlanner info)
   {
      this.scale = scaleF;
      xUpper = info.getRRTTree().upperBoundNode.getNodeData(0);
      xLower = info.getRRTTree().lowerBoundNode.getNodeData(0);
      yUpper = info.getRRTTree().upperBoundNode.getNodeData(1);
      yLower = info.getRRTTree().lowerBoundNode.getNodeData(1);

      sizeU = (int) Math.round((-yLower + yUpper) * scale);
      sizeV = (int) Math.round((-xLower + xUpper) * scale);

      allPoints = pointInfo;
      this.numOfPoints = allPoints.length / 4;

      //pathPoints = pointInfoOfPath;
      boxInfo[0] = RRT2DNode.boxCenterX;
      boxInfo[1] = RRT2DNode.boxCenterY;
      boxInfo[2] = RRT2DNode.boxSizeX;
      boxInfo[3] = RRT2DNode.boxSizeY;
      this.info = info;
   }

   public void paint(Graphics g)
   {
      super.paint(g);
      g.setColor(Color.red);
      g.drawLine(y2u(0.0), x2v(0.0), y2u(0.5), x2v(0.0));
      g.setColor(Color.green);
      g.drawLine(y2u(0.0), x2v(0.0), y2u(0.0), x2v(0.5));

      g.setColor(Color.GRAY);

      // all node and branch line
      for (int i = 0; i < numOfPoints; i++)
      {
         branchFill(g, allPoints[i * 4 + 0], allPoints[i * 4 + 1], allPoints[i * 4 + 2], allPoints[i * 4 + 3], 4);
      }

      // root node
      point(g, 0.0, 0.0, 4);

      // path
      g.setColor(Color.MAGENTA);      
      ArrayList<RRTNode> infoPathNode = info.getRRTTree().pathNode;
      for (int i =1;i<infoPathNode.size();i++)
      {
         branch(g, infoPathNode.get(i-1).getNodeData(0), infoPathNode.get(i-1).getNodeData(1), infoPathNode.get(i).getNodeData(0), infoPathNode.get(i).getNodeData(1), 6);
      }

      // goal node
      g.setColor(Color.blue);
      point(g, info.getGoalNode().getNodeData(0), info.getGoalNode().getNodeData(1), 4);

//      g.setColor(Color.red);      
      // box
      branch(g, boxInfo[0] + boxInfo[2] * 0.5, boxInfo[1] + boxInfo[3] * 0.5, boxInfo[0] + boxInfo[2] * 0.5, boxInfo[1] - boxInfo[3] * 0.5, 2);
      branch(g, boxInfo[0] + boxInfo[2] * 0.5, boxInfo[1] + boxInfo[3] * 0.5, boxInfo[0] - boxInfo[2] * 0.5, boxInfo[1] + boxInfo[3] * 0.5, 2);
      branch(g, boxInfo[0] - boxInfo[2] * 0.5, boxInfo[1] - boxInfo[3] * 0.5, boxInfo[0] - boxInfo[2] * 0.5, boxInfo[1] + boxInfo[3] * 0.5, 2);
      branch(g, boxInfo[0] - boxInfo[2] * 0.5, boxInfo[1] - boxInfo[3] * 0.5, boxInfo[0] + boxInfo[2] * 0.5, boxInfo[1] - boxInfo[3] * 0.5, 2);

      // piecewise
      g.setColor(Color.green);      
      ArrayList<RRTNode> piecewisePath = info.getPiecewisePath().getPiecewisePath();      
      for (int i =0;i< piecewisePath.size()-1;i++)
      {
         double p1x = piecewisePath.get(i).getNodeData(0);
         double p1y = piecewisePath.get(i).getNodeData(1);
         double p2x = piecewisePath.get(i+1).getNodeData(0);
         double p2y = piecewisePath.get(i+1).getNodeData(1);
         branch(g, p1x, p1y, p2x, p2y, 4);
      }
      
      // shortcut
      g.setColor(Color.black);      
      ArrayList<RRTNode> optimalPath = info.getOptimalPath();
      for (int i =0;i< optimalPath.size()-1;i++)
      {
         double p1x = optimalPath.get(i).getNodeData(0);
         double p1y = optimalPath.get(i).getNodeData(1);
         double p2x = optimalPath.get(i+1).getNodeData(0);
         double p2y = optimalPath.get(i+1).getNodeData(1);
         branchFill(g, p1x, p1y, p2x, p2y, 6);
      }
   }

   public int x2v(double px)
   {
      return (int) Math.round(((-px) + xUpper) * scale);
   }

   public int y2u(double py)
   {
      return (int) Math.round(((-py) + yUpper) * scale);
   }
   
   public void branch(Graphics g, double p1x, double p1y, double p2x, double p2y, int size)
   {
      point(g, p1x, p1y, size);
      point(g, p2x, p2y, size);

      g.drawLine(y2u(p1y), x2v(p1x), y2u(p2y), x2v(p2x));
   }
   
   public void branchFill(Graphics g, double p1x, double p1y, double p2x, double p2y, int size)
   {
      pointFill(g, p1x, p1y, size);
      pointFill(g, p2x, p2y, size);

      g.drawLine(y2u(p1y), x2v(p1x), y2u(p2y), x2v(p2x));
   }
   
   public void point(Graphics g, double px, double py, int size)
   {
      int diameter = size;
      g.drawOval(y2u(py) - diameter / 2, x2v(px) - diameter / 2, diameter, diameter);
   }
   
   public void pointFill(Graphics g, double px, double py, int size)
   {
      int diameter = size;
      g.fillOval(y2u(py) - diameter / 2, x2v(px) - diameter / 2, diameter, diameter);
   }
}

public class RRTTestProject
{
   public static void main(String[] args)
   {
      PrintTools.info("Start!!! ");

      RRTNode startNode = new RRT2DNode(0.0, 0.0);
      RRTNode goalNode = new RRT2DNode(3.0, 2.0);
      RRT2DPlanner rrtPlanner = new RRT2DPlanner(startNode, goalNode, 0.4);

      RRTNode upperBoundNode = new RRT2DNode(5.0, 4.0);
      RRTNode lowerBoundNode = new RRT2DNode(-5.0, -4.0);
      rrtPlanner.getRRTTree().setUpperBound(upperBoundNode);
      rrtPlanner.getRRTTree().setLowerBound(lowerBoundNode);

      int maxNumberOfExpanding = 1500;
      double[] rrtPointInfo;
      rrtPointInfo = new double[maxNumberOfExpanding * 4];
      double[] rrtNewPointInfo;
      rrtNewPointInfo = new double[4];

      for (int i = 0; i < maxNumberOfExpanding; i++)
      {  
         for (int j = 0; j < 4; j++)
         {
            rrtPointInfo[i * 4 + j] = rrtNewPointInfo[j];
         }

         if (rrtPlanner.expandTreeGoal(rrtNewPointInfo) == true)
         {
            i++;
            for (int j = 0; j < 4; j++)
            {
               rrtPointInfo[i * 4 + j] = rrtNewPointInfo[j];
            }
            
            i++;
            for (int j = 0; j < 2; j++)
            {               
               rrtPointInfo[i * 4 + j] = rrtPointInfo[(i-1) * 4 + j];
               rrtPointInfo[i * 4 + 2+j] = rrtPlanner.getGoalNode().getNodeData(j);
            }
            break;
         }
      }

      JFrame frame;
      DrawPanel drawPanel;
      Dimension dim;
      
//      String newAmount = new Scanner(System.in).nextLine();
      
      rrtPlanner.updateOptimalPath(101, 100);

      frame = new JFrame("RRTTest");
      drawPanel = new DrawPanel(100, rrtPointInfo, rrtPlanner);
      dim = new Dimension(drawPanel.sizeU, drawPanel.sizeV);
      frame.setPreferredSize(dim);
      frame.setLocation(200, 100);

      frame.add(drawPanel);
      frame.pack();
      frame.setVisible(true);
      
      
      
      
   }
}
