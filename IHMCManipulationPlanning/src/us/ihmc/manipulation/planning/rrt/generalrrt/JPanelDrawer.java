package us.ihmc.manipulation.planning.rrt.generalrrt;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class JPanelDrawer extends JPanel
{
   public double xBoundary[] = {0, 0};
   public double yBoundary[] = {0, 0};

   private double scale = 100;

   private int sizeU = 1000;
   private int sizeV = 1000;
   
   private static final long serialVersionUID = 1L;

   public ArrayList<RRTNode> validNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> invalidNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> pathNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> optimalNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> piecewiseNodes = new ArrayList<RRTNode>();
   
   
   public JPanelDrawer()
   {
      
   }
   

   public void paint(Graphics g)
   {
      super.paint(g);
      g.setColor(Color.red);
      g.drawLine(y2u(0.0), x2v(0.0), y2u(0.5), x2v(0.0));
      g.setColor(Color.green);
      g.drawLine(y2u(0.0), x2v(0.0), y2u(0.0), x2v(0.5));

      /*
       * valid Nodes
       */
      g.setColor(Color.GRAY);
      for (int i = 0; i < validNodes.size(); i++)
      {
         if(validNodes.get(i).getParentNode() != null)
         {
            branchFill(g, validNodes.get(i).getNodeData(0), validNodes.get(i).getNodeData(1), validNodes.get(i).getParentNode().getNodeData(0), validNodes.get(i).getParentNode().getNodeData(1), 4);   
         }
         else
         {
            point(g, validNodes.get(i).getNodeData(0), validNodes.get(i).getNodeData(1), 6);
         }         
      }
      
      /*
       * valid path Nodes
       */
      g.setColor(Color.MAGENTA);
      for (int i =1;i<pathNodes.size();i++)
      {
         branch(g, pathNodes.get(i-1).getNodeData(0), pathNodes.get(i-1).getNodeData(1), pathNodes.get(i).getNodeData(0), pathNodes.get(i).getNodeData(1), 6);
      }

      /*
       * invalid Nodes
       */
      g.setColor(Color.RED);
      for (int i =1;i<invalidNodes.size();i++)
      {
         point(g, invalidNodes.get(i).getNodeData(0), invalidNodes.get(i).getNodeData(1), 4);
      }

      /*
       * invalid region
       */
      g.setColor(Color.red);      
      branch(g, RRTNode2D.boxCenterX + RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY + RRTNode2D.boxSizeY * 0.5, RRTNode2D.boxCenterX + RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY - RRTNode2D.boxSizeY * 0.5, 2);
      branch(g, RRTNode2D.boxCenterX + RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY + RRTNode2D.boxSizeY * 0.5, RRTNode2D.boxCenterX - RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY + RRTNode2D.boxSizeY * 0.5, 2);
      branch(g, RRTNode2D.boxCenterX - RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY - RRTNode2D.boxSizeY * 0.5, RRTNode2D.boxCenterX - RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY + RRTNode2D.boxSizeY * 0.5, 2);
      branch(g, RRTNode2D.boxCenterX - RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY - RRTNode2D.boxSizeY * 0.5, RRTNode2D.boxCenterX + RRTNode2D.boxSizeX * 0.5, RRTNode2D.boxCenterY - RRTNode2D.boxSizeY * 0.5, 2);

      /*
       * piecewise
       */
      g.setColor(Color.green);
      for (int i =0;i< piecewiseNodes.size()-1;i++)
      {
         double p1x = piecewiseNodes.get(i).getNodeData(0);
         double p1y = piecewiseNodes.get(i).getNodeData(1);
         double p2x = piecewiseNodes.get(i+1).getNodeData(0);
         double p2y = piecewiseNodes.get(i+1).getNodeData(1);
         branch(g, p1x, p1y, p2x, p2y, 4);
      }

      /*
       * optimal
       */
      g.setColor(Color.black);
      for (int i =0;i< optimalNodes.size()-1;i++)
      {
         double p1x = optimalNodes.get(i).getNodeData(0);
         double p1y = optimalNodes.get(i).getNodeData(1);
         double p2x = optimalNodes.get(i+1).getNodeData(0);
         double p2y = optimalNodes.get(i+1).getNodeData(1);
         branchFill(g, p1x, p1y, p2x, p2y, 6);
      }
   }
   
   public void draw()
   {
      JFrame frame;
      Dimension dim;
      
      frame = new JFrame("RRT");
      
      dim = new Dimension(this.sizeU, this.sizeV);
      frame.setPreferredSize(dim);
      frame.setLocation(200, 100);

      frame.add(this);
      frame.pack();
      frame.setVisible(true);   
   }
   
   public int x2v(double px)
   {
      return (int) Math.round(((-px) + xBoundary[1]) * scale);
   }

   public int y2u(double py)
   {
      return (int) Math.round(((-py) + yBoundary[1]) * scale);
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
