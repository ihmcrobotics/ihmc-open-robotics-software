package us.ihmc.simulationConstructionSetTools.util.inputdevices.sliderboardVisualizer;

import javax.swing.ImageIcon;
import javax.swing.JLabel;

public class VisualizerIcon
{
   private final JLabel image;
   private final JLabel label;
   private final JLabel max;
   private final JLabel min;
   private final JLabel currentValue;

   public VisualizerIcon(ImageIcon imageIcon)
   {
      image = new JLabel(imageIcon);
      label = new JLabel("");
      max = new JLabel("");
      min = new JLabel("");
      currentValue = new JLabel("");
   }

   public void clear()
   {
      label.setText("");
      max.setText("");
      min.setText("");
      currentValue.setText("");
   }

   public void setImageSize(int x, int y)
   {
      image.setSize(x, y);
   }

   public void setImageLocation(int x, int y)
   {
      image.setLocation(x, y);
   }

   public void setLabelBounds(int x, int y, int width, int height)
   {
      label.setBounds(x, y, width, height);
   }

   public void setLabelText(String text)
   {
      label.setText(text);
   }

   public void setMaxBounds(int x, int y, int width, int height)
   {
      max.setBounds(x, y, width, height);
   }

   public void setMinBounds(int x, int y, int width, int height)
   {
      min.setBounds(x, y, width, height);
   }

   public void setMaxMinValues(double max, double min)
   {
      this.max.setText("" + max);
      this.min.setText("" + min);
   }

   public void clearMaxMinValues()
   {
      max.setText("");
      min.setText("");
   }

   public void setCurrentValueBounds(int x, int y, int width, int height)
   {
      currentValue.setBounds(x, y, width, height);
   }

   public void clearCurrentValue()
   {
      currentValue.setText("");
   }

   public void updateCurrentValue(double currentValue)
   {
      this.currentValue.setText("" + currentValue);
   }

   public void updateCurrentValue(String currentValue)
   {
      this.currentValue.setText(currentValue);
   }

   public JLabel getImage()
   {
      return image;
   }

   public JLabel getLabel()
   {
      return label;
   }

   public JLabel getMax()
   {
      return max;
   }

   public JLabel getMin()
   {
      return min;
   }


   public JLabel getCurrentValue()
   {
      return currentValue;
   }
}
