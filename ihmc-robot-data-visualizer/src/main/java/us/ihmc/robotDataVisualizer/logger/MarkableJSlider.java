package us.ihmc.robotDataVisualizer.logger;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JSlider;
import javax.swing.plaf.basic.BasicSliderUI;

public class MarkableJSlider extends JSlider
{
   private static final long serialVersionUID = 7020394143203655524L;

   private int start = -1;
   private int end = -1;

   public MarkableJSlider(int min, int max, int value)
   {
      super(min, max, value);
      setUI(new MarkableSliderUI(this));
   }

   public void markStart()
   {
      start = getValue();
      if (end != -1 && start >= end)
      {
         end = start + 1;
      }
      repaint();
   }

   public void markEnd()
   {
      end = getValue();
      if (end != -1 && end <= start)
      {
         start = end - 1;
      }
      repaint();

   }
   
   public void clear()
   {
      start = -1;
      end = -1;
      repaint();
   }
   
   public int getStart()
   {
      return start;
   }
   
   public int getEnd()
   {
      return end;
   }

   public class MarkableSliderUI extends BasicSliderUI
   {

      public MarkableSliderUI(JSlider slider)
      {
         super(slider);
      }

      @Override
      public void paint(Graphics g, JComponent c)
      {
         super.paint(g, c);

         int w = 4;
         int h = thumbRect.height;

         if(start != -1)
         {
            int x = xPositionForValue(start) - w/2;
            g.translate(x, thumbRect.y);
   
            g.setColor(Color.green);
            g.fillRect(0, 0, w, h);
   
            g.setColor(Color.black);
            g.drawLine(0, h - 1, w - 1, h - 1);
            g.drawLine(w - 1, 0, w - 1, h - 1);
            g.translate(-x, -thumbRect.y);
         }
         if(end != -1)
         {
            int x = xPositionForValue(end) - w/2;
            g.translate(x, thumbRect.y);
            
            g.setColor(Color.red);
            g.fillRect(0, 0, w, h);
            
            g.setColor(Color.black);
            g.drawLine(0, h - 1, w - 1, h - 1);
            g.drawLine(w - 1, 0, w - 1, h - 1);
            g.translate(-x, -thumbRect.y);
         }
      }

   }

   public static void main(String[] args)
   {
      final MarkableJSlider slider = new MarkableJSlider(0, 500, 50);
      JButton markStart = new JButton("Start");
      markStart.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.markStart();
         }

      });

      JButton markEnd = new JButton("End");
      markEnd.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.markEnd();
         }
      });
      
      JButton clear = new JButton("Clear");
      clear.addActionListener(new ActionListener()
      {
         
         @Override
         public void actionPerformed(ActionEvent e)
         {
            slider.clear();
         }
      });

      JFrame frame = new JFrame();
      frame.getContentPane().setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.X_AXIS));
      frame.getContentPane().add(slider);
      frame.getContentPane().add(markStart);
      frame.getContentPane().add(markEnd);
      frame.getContentPane().add(clear);
      frame.setMinimumSize(new Dimension(800, 50));
      frame.setLocationRelativeTo(null);
      frame.setLocationByPlatform(true);
      frame.setVisible(true);

      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
   }
}
