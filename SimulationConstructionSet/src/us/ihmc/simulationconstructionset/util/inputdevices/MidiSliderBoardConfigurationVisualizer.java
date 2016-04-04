package us.ihmc.simulationconstructionset.util.inputdevices;

import javax.swing.*;
import java.awt.*;
import java.util.HashMap;
import java.util.Hashtable;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiControl.ControlType;

public class MidiSliderBoardConfigurationVisualizer extends JFrame
{
   private Hashtable<Integer, MidiControl> controlsHashTable = new Hashtable<Integer, MidiControl>(40);

   private HashMap<Integer, JLabel> buttonMap = new HashMap<>();
   private HashMap<Integer, JLabel> knobMap = new HashMap<>();
   private HashMap<Integer, JLabel> sliderMap = new HashMap<>();

   private HashMap<JLabel, JLabel> buttonGroup = new HashMap<>();
   private HashMap<JLabel, JLabel> knobGroup = new HashMap<>();
   private HashMap<JLabel, JLabel> sliderGroup = new HashMap<>();

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

      panel.setPreferredSize(new Dimension(1000, 600));
      createControls();

      this.getContentPane().add(panel);
      this.pack();
   }

   public void clearLabels()
   {
      for (JLabel label : buttonGroup.values())
         label.setText("Unassigned");
      for (JLabel label : knobGroup.values())
         label.setText("Unassigned");
      for (JLabel label : sliderGroup.values())
         label.setText("Unassigned");
      this.setTitle("Unassigned");
   }

   public void createLabels()
   {
      for (MidiControl tmpControl : controlsHashTable.values())
      {
         int mapping = tmpControl.mapping - 1;
         if (tmpControl.controlType == ControlType.BUTTON)
         {
            JLabel label = buttonGroup.get(buttonMap.get(mapping));
            if (label != null)
               label.setText(tmpControl.name);
            else
               System.out.println("Button requested does not exist on the board.");
         }
         else if (tmpControl.controlType == ControlType.SLIDER)
         {
            JLabel label = sliderGroup.get(sliderMap.get(mapping));
            if (label != null)
               label.setText(tmpControl.name);
            else
               System.out.println("Slider requested does not exist on the board.");
         }
         else if (tmpControl.controlType == ControlType.KNOB)
         {
            JLabel label = knobGroup.get(knobMap.get(mapping));
            if (label != null)
               label.setText(tmpControl.name);
            else
               System.out.println("Knob requested does not exist on the board.");
         }
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
      JLabel knob = new JLabel(knobImage);
      knob.setSize(75, 75);
      knob.setLocation(125 * column + 13, 100 * row + 13);

      JLabel label = new JLabel("Unassigned");
      label.setBounds(125 * column, 100 * row, 125, 13);

      this.add(knob);
      this.add(label);

      knobGroup.put(knob, label);
      knobMap.put(knobIndex - 80, knob);
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

      JLabel button = new JLabel(buttonImage);
      button.setSize(50, 30);
      button.setLocation(125 * column + 25, 50 * row + 115);

      JLabel label = new JLabel("Unassigned");
      label.setBounds(125 * column, 50 * row + 100, 125, 13);

      this.add(button);
      this.add(label);

      buttonGroup.put(button, label);
      buttonMap.put(buttonIndex + offset, button);
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
      JLabel slider = new JLabel(sliderImage);
      slider.setSize(80, 362);
      slider.setLocation(125 * column + 10, 362 * row + 215);

      JLabel label = new JLabel("Unassigned");
      label.setBounds(125 * column, 100 * row + 200, 125, 13);

      this.add(slider);
      this.add(label);

      sliderGroup.put(slider, label);
      sliderMap.put(sliderIndex, slider);
      sliderIndex++;
   }

   public static void main(String[] args)
   {
      MidiSliderBoardConfigurationVisualizer visualizer = new MidiSliderBoardConfigurationVisualizer(null);
   }
}
