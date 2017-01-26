package us.ihmc.simulationconstructionset.util.inputdevices.sliderboardVisualizer;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.Hashtable;

import us.ihmc.simulationconstructionset.util.inputdevices.MidiControl;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiControl.SliderType;

@SuppressWarnings("serial")
public class MidiSliderBoardConfigurationVisualizer extends JFrame
{
   private Hashtable<Integer, MidiControl> controlsHashTable = new Hashtable<Integer, MidiControl>(40);

   private HashMap<Integer, VisualizerIcon> iconMap = new HashMap<>();

   private int buttonIndex = 0;
   private int knobIndex = 0;
   private int sliderIndex = 0;

   private final static ImageIcon buttonImage = new ImageIcon(ClassLoader.getSystemResource("images/MidiButton.jpeg").getPath());
   private final static ImageIcon sliderImage = new ImageIcon(ClassLoader.getSystemResource("images/MidiSlider.jpg").getPath());
   private final static ImageIcon knobImage = new ImageIcon(ClassLoader.getSystemResource("images/MidiKnob.jpg").getPath());

   private static final int rowsOfKnobs = 1;
   private static final int columnsOfKnobs = 8;

   private static final int rowsOfButtons = 2;
   private static final int columnsOfButtons = 8;

   private static final int rowsOfSliders = 1;
   private static final int columnsOfSliders = 8;

   private final JPanel panel = new JPanel();

   public MidiSliderBoardConfigurationVisualizer(Hashtable<Integer, MidiControl> controlsHashTable)
   {
      super("Midi SliderBoard");

      this.controlsHashTable = controlsHashTable;

      this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      this.setVisible(true);

      panel.setPreferredSize(new Dimension(1200, 700));
      createControls();

      this.getContentPane().add(panel);
      this.pack();
   }

   public void clearLabels()
   {
      for (VisualizerIcon icon : iconMap.values())
         icon.clear();
      this.setTitle("Unassigned");
   }

   public void updateValues()
   {
      for (MidiControl tmpControl : controlsHashTable.values())
      {
         int mapping = tmpControl.mapping - 1;
         VisualizerIcon icon = iconMap.get(mapping);
         if (icon != null)
            icon.updateCurrentValue(tmpControl.var.getValueAsDouble());
         else
            System.out.println("Icon requested does not exist on the board.");
      }
   }

   public void updateValue(MidiControl tmpControl)
   {
      int mapping = tmpControl.mapping - 1;
      VisualizerIcon icon = iconMap.get(mapping);
      if (icon != null)
      {
         if (tmpControl.sliderType.equals(SliderType.BOOLEAN))
         {
            if (tmpControl.var.getValueAsDouble() == 0)
               icon.updateCurrentValue("false");
            else
               icon.updateCurrentValue("true");
         }
         else
            icon.updateCurrentValue(tmpControl.var.getValueAsDouble());
      }
      else
         System.out.println("Icon requested does not exist on the board.");
   }

   public void updateValue(MidiControl tmpControl, double value)
   {
      int mapping = tmpControl.mapping - 1;
      VisualizerIcon icon = iconMap.get(mapping);
      if (icon != null)
         icon.updateCurrentValue(value);
      else
         System.out.println("Icon requested does not exist on the board.");
   }

   public void createLabels()
   {
      for (MidiControl tmpControl : controlsHashTable.values())
      {
         int mapping = tmpControl.mapping - 1;
         VisualizerIcon icon = iconMap.get(mapping);
         if (icon != null)
         {
            icon.setLabelText(tmpControl.name);
            icon.setMaxMinValues(tmpControl.max, tmpControl.min);

            if (tmpControl.sliderType.equals(SliderType.BOOLEAN))
            {
               if (tmpControl.currentVal == 0)
                  icon.updateCurrentValue("false");
               else
                  icon.updateCurrentValue("true");
            }
            else
               icon.updateCurrentValue(tmpControl.currentVal);
         }
         else
            System.out.println("Icon requested does not exist on the board.");
      }
   }

   private void createControls()
   {
      createKnobs();
      createSliders();
      createButtons();
   }

   private void createKnobs()
   {
      for (int row = 0; row < rowsOfKnobs; row++)
      {
         for (int column = 0; column < columnsOfKnobs; column++)
            createKnob(row, column);
      }
   }

   private void createKnob(int row, int column)
   {
      VisualizerIcon knob = new VisualizerIcon(knobImage);
      knob.setImageSize(75, 75);
      knob.setImageLocation(150 * column + 38, 125 * row + 13);
      knob.setLabelBounds(150 * column, 100 * row, 125, 13);
      knob.setMaxBounds(150 * column + 87, 100 * row + 88, 50, 13);
      knob.setMinBounds(150 * column + 27, 100 * row + 88, 50, 13);
      knob.setCurrentValueBounds(150 * column + 57, 100 * row + 101, 100, 13);

      this.add(knob.getImage());
      this.add(knob.getLabel());
      this.add(knob.getMax());
      this.add(knob.getMin());
      this.add(knob.getCurrentValue());

      iconMap.put(knobIndex - 80, knob);
      knobIndex++;
   }

   private void createButtons()
   {
      for (int row = 0; row < rowsOfButtons; row++)
      {
         for (int column = 0; column < columnsOfButtons; column++)
            createButton(row, column);
      }
   }

   private void createButton(int row, int column)
   {
      int offset;
      if ((buttonIndex >= 17) && (buttonIndex <= 20))
         offset = -8;
      else
         offset = -16;

      VisualizerIcon button = new VisualizerIcon(buttonImage);
      button.setImageSize(50, 30);
      button.setImageLocation(150 * column + 50, 50 * row + 140);
      button.setLabelBounds(150 * column + 13, 50 * row + 125, 125, 13);
      button.setCurrentValueBounds(150 * column + 13, 50 * row + 138, 50, 13);

      this.add(button.getImage());
      this.add(button.getLabel());
      this.add(button.getCurrentValue());

      iconMap.put(buttonIndex + offset, button);
      buttonIndex++;
   }

   private void createSliders()
   {
      for (int row = 0; row < rowsOfSliders; row++)
      {
         for (int column = 0; column < columnsOfSliders; column++)
            createSlider(row, column);
      }
   }

   private void createSlider(int row, int column)
   {
      VisualizerIcon slider = new VisualizerIcon(sliderImage);
      slider.setImageSize(80, 362);
      slider.setImageLocation(150 * column + 30, 362 * row + 240);
      slider.setLabelBounds(150 * column + 50, 100 * row + 225, 125, 13);
      slider.setMaxBounds(150 * column, 362 * row + 240, 50, 13);
      slider.setMinBounds(150 * column, 362 * row + 602, 50, 13);
      slider.setCurrentValueBounds(150 * column + 50, 362 * row + 615, 100, 13);

      this.add(slider.getImage());
      this.add(slider.getLabel());
      this.add(slider.getMax());
      this.add(slider.getMin());
      this.add(slider.getCurrentValue());

      iconMap.put(sliderIndex, slider);
      sliderIndex++;
   }

   public static void main(String[] args)
   {
      new MidiSliderBoardConfigurationVisualizer(null);
   }
}
