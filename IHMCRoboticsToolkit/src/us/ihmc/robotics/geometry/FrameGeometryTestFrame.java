package us.ihmc.robotics.geometry;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class FrameGeometryTestFrame extends JFrame
{
   private static final long serialVersionUID = 1211077223435215178L;
   private final FrameGeometry2dPlotter frameGeometry2dPlotter;
   private final GoodButton goodButton;
   private final BadButton badButton;

   public FrameGeometryTestFrame(double xMin, double xMax, double yMin, double yMax)
   {
      this("Test", xMin, xMax, yMin, yMax);
   }

   public FrameGeometryTestFrame(String name, double xMin, double xMax, double yMin, double yMax)
   {
      super(name);

      double scale = 800.0 / Math.max(xMax - xMin, yMax - yMin);

      frameGeometry2dPlotter = new FrameGeometry2dPlotter((xMin + xMax) / 2.0, (yMin + yMax) / 2.0, scale);

      goodButton = new GoodButton();
      badButton = new BadButton();

      JPanel buttonPanel = new JPanel(new FlowLayout());
      buttonPanel.add(goodButton);
      buttonPanel.add(badButton);

      add(buttonPanel, BorderLayout.SOUTH);

      add(frameGeometry2dPlotter, BorderLayout.CENTER);
      setSize(800, 800);
      setVisible(true);
   }

   public FrameGeometry2dPlotter getFrameGeometry2dPlotter()
   {
      return frameGeometry2dPlotter;
   }

   public void addTestPoints(ArrayList<FramePoint2d> testPoints)
   {
      frameGeometry2dPlotter.addTestPoints(testPoints);
   }

   public boolean hasGoodButtonBeenPressed()
   {
      return goodButton.hasBeenPressed();
   }

   public boolean hasBadButtonBeenPressed()
   {
      return badButton.hasBeenPressed();
   }

   public void waitForButtonPush()
   {
      while (!hasGoodButtonBeenPressed() &&!hasBadButtonBeenPressed())
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }

      if (hasBadButtonBeenPressed())
         throw new RuntimeException();
   }


   private class GoodButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 6028873461564012484L;
      private boolean hasBeenPressed = false;

      public GoodButton()
      {
         super("Good");

         this.addActionListener(this);
      }

      public void actionPerformed(ActionEvent e)
      {
         hasBeenPressed = true;
      }

      public boolean hasBeenPressed()
      {
         return hasBeenPressed;
      }
   }


   private class BadButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 8505700581984408817L;
      private boolean hasBeenPressed = false;

      public BadButton()
      {
         super("Bad");

         this.addActionListener(this);
      }

      public void actionPerformed(ActionEvent e)
      {
         hasBeenPressed = true;
      }

      public boolean hasBeenPressed()
      {
         return hasBeenPressed;
      }
   }
}
