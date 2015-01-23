package us.ihmc.plotting;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.vecmath.Point2d;

import us.ihmc.plotting.shapes.PointArtifact;

public class ISS_Simulator
{
   public static void main(String[] args)
   {
      PlotterPanel p = new PlotterPanel();

      BufferedImage image = null;
      try
      {
         image = ImageIO.read(new File("./resources/ISS.jpg"));
      }
      catch (IOException e)
      {
         JOptionPane.showMessageDialog(null, e.getMessage(), "", JOptionPane.ERROR_MESSAGE);
         System.exit(1);
      }

      p.getPlotter().setBackgroundImage(image);
      p.getPlotter().setRangeLimit(20, (double) (image.getWidth() / 20.0), -10.0, 10.0, 10.0, -10.0);


      JFrame f = new JFrame("Plotter Panel");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });

//    f.setJMenuBar((JMenuBar)p.getMenus().get(0));
      f.getContentPane().add(new JScrollPane(p), BorderLayout.CENTER);
      f.pack();
      f.setVisible(true);

//    for (float i = 0; i < 100; i++)
//    {
//      try
//      {
//         Thread.sleep(100);
//      }
//      catch (InterruptedException e1)
//      {
//         e1.printStackTrace();
//      }

//      PointArtifact pa = new PointArtifact("test_" + i, new Point2d(i / 10, 0));
//      p.getPlotter().addArtifact(pa);


      PointArtifact pa = new PointArtifact("test1", new Point2d(0, 0));
      pa.setColor(Color.red);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.11", new Point2d(-7.8, 2.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.12", new Point2d(-7.8, 2.8));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.21", new Point2d(-5.3, 2.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.22", new Point2d(-5.3, 2.8));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.3", new Point2d(-2.5, 2.5));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.4", new Point2d(0.0, 2.5));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test1.1", new Point2d(-0.7, 3.3));
      pa.setColor(Color.red);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test1.2", new Point2d(-0.3, 3.3));
      pa.setColor(Color.green);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test1.3", new Point2d(0.3, 3.3));
      pa.setColor(Color.blue);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test1.4", new Point2d(0.7, 3.3));
      pa.setColor(Color.yellow);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test5.1", new Point2d(-0.7, -7.8));
      pa.setColor(Color.red);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test5.2", new Point2d(-0.3, -7.8));
      pa.setColor(Color.green);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test5.3", new Point2d(0.3, -7.8));
      pa.setColor(Color.blue);
      p.getPlotter().addArtifact(pa);
      pa = new PointArtifact("test5.4", new Point2d(0.7, -7.8));
      pa.setColor(Color.yellow);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.5", new Point2d(2.5, 2.5));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.61", new Point2d(5.0, 2.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test2.62", new Point2d(5.0, 2.8));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.1", new Point2d(0.0, 0.0));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.2", new Point2d(-0.5, -1.0));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.3", new Point2d(0.5, -1.0));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.4", new Point2d(-0.5, -2.7));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.5", new Point2d(0.5, -2.7));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test3.6", new Point2d(0.0, -3.5));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test4.1", new Point2d(0.0, -6.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test4.2", new Point2d(3.0, -6.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test4.3", new Point2d(-3.0, -6.2));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test4.4", new Point2d(-4.3, -5.8));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

      pa = new PointArtifact("test4.5", new Point2d(-4.3, -6.7));
      pa.setColor(Color.gray);
      p.getPlotter().addArtifact(pa);

//    }
   }
}
