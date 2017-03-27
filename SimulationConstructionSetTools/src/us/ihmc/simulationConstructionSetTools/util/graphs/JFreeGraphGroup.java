package us.ihmc.simulationConstructionSetTools.util.graphs;

import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.awt.image.RenderedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.border.BevelBorder;

public class JFreeGraphGroup extends JPanel
{
   ArrayList<JFreeGraph> graphs = new ArrayList<JFreeGraph>();

   final int graphsPerRow = 3;


   public JFreeGraphGroup()
   {
      setUpJpanel();
   }

   public JFreeGraphGroup(ArrayList<JFreeGraph> graphs)
   {
      this.graphs = graphs;
      setUpJpanel();
   }

   public void addGraph(JFreeGraph graph)
   {
      graphs.add(graph);
      setUpJpanel();
   }

   private void setUpJpanel()
   {
      this.removeAll();

      int rows = graphs.size() / graphsPerRow;

      this.setLayout(new GridLayout(rows, graphsPerRow));

      int i = 0;
      for (JFreeGraph graph : graphs)
      {
         graph.setBorder(new BevelBorder(2));
         this.add(graph);
         i++;
      }

      this.repaint();
   }

   public void saveToJPEG(String fileName)
   {
      int width = 4800, height = 4600;
      JFrame frame = new JFrame("Saving");
      JFreeGraphGroup group2 = new JFreeGraphGroup();

      // group2.setSize(width, height);

      for (JFreeGraph tmpGraph : graphs)
      {
         JFreeGraph tmp = tmpGraph.clone();

         group2.addGraph(tmp);
         tmp.setSize(400, 400);
         System.out.println(tmp.getSize());
      }

      frame.add(group2);
      frame.setSize(width, height);
      frame.setVisible(true);
      BufferedImage image = new BufferedImage(group2.getWidth(), group2.getHeight(), BufferedImage.TYPE_INT_RGB);

      group2.paint(image.getGraphics());

      RenderedImage rendImage = image;

      // Write generated image to a file

      if (!(fileName.endsWith(".jpg") || fileName.endsWith(".jpeg")))
      {
         fileName += ".jpg";
      }

      try
      {
         File file = new File(fileName);

         // Save as PNG


         System.out.println("saving jpg");

         ImageIO.write(rendImage, "jpg", file);
         Thread.sleep(1000);
         System.out.println("done");

      }
      catch (Exception e)
      {
      }

      frame.setVisible(false);

      // / setUpJpanel();

   }

   public void saveToPNG(String fileName)
   {
      int width = 4800, height = 4600;
      JFrame frame = new JFrame("Saving");
      JFreeGraphGroup group2 = new JFreeGraphGroup();

      // group2.setSize(width, height);

      for (JFreeGraph tmpGraph : graphs)
      {
         JFreeGraph tmp = tmpGraph.clone();

         group2.addGraph(tmp);
         tmp.setSize(400, 400);
         System.out.println(tmp.getSize());
      }

      frame.add(group2);
      frame.setSize(width, height);
      frame.setVisible(true);
      BufferedImage image = new BufferedImage(group2.getWidth(), group2.getHeight(), BufferedImage.TYPE_INT_RGB);

      group2.paint(image.getGraphics());

      RenderedImage rendImage = image;

      // Write generated image to a file

      if (!(fileName.endsWith(".png")))
      {
         fileName += ".png";
      }

      try
      {
         File file = new File(fileName);

         // Save as PNG

         System.out.println("saving png");

         ImageIO.write(rendImage, "png", file);

         Thread.sleep(1000);
         System.out.println("done");

      }
      catch (Exception e)
      {
      }

      frame.setVisible(false);

      // / setUpJpanel();

   }

}
