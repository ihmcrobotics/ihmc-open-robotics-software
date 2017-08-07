package us.ihmc.simulationconstructionset.gui;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class HumanAssistedTestFrame extends JFrame
{
   private static final long serialVersionUID = -6270384898459053226L;
   private final GoodButton goodButton;
   private final BadButton badButton;

   // todo: Make nicities, like inputting information from the human in order to check a test case, easier ways to draw and plot things, etc, etc.

   public HumanAssistedTestFrame(String name)
   {
      JFrame frame = new JFrame(name);

      goodButton = new GoodButton();
      badButton = new BadButton();

      JPanel buttonPanel = new JPanel(new FlowLayout());
      buttonPanel.add(goodButton);
      buttonPanel.add(badButton);

      frame.add(buttonPanel, BorderLayout.SOUTH);

      frame.setSize(800, 800);
      frame.setVisible(true);
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
      /**
       *
       */
      private static final long serialVersionUID = 3222645729570934059L;
      private boolean hasBeenPressed = false;

      public GoodButton()
      {
         super("Good");

         this.addActionListener(this);
      }

      @Override
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
      /**
       *
       */
      private static final long serialVersionUID = -4648731010106093744L;
      private boolean hasBeenPressed = false;

      public BadButton()
      {
         super("Bad");

         this.addActionListener(this);
      }

      @Override
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
