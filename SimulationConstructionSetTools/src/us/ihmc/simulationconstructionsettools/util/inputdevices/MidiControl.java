package us.ihmc.simulationconstructionsettools.util.inputdevices;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class MidiControl
{
   public int mapping = -1;
   public double reset = -1;
   public double max = 127;
   public double min = 0;
   public double exponent = 1;
   public double hires = (min+max)/2.0;
   public double currentVal = -1;
   public YoVariable<?> var = null;
   public String name = null;

   public enum SliderType
   {
      BOOLEAN, ENUM, NUMBER, INT
   }

   public SliderType sliderType;

   public enum ControlType
   {
      SLIDER, BUTTON, KNOB
   }

   public ControlType controlType;
   public boolean notify = true;

   public MidiControl(int mapping, YoVariable<?> var, double max, double min, double exponent)
   {
      this(mapping, var, max, min, exponent, (min+max)/2.0, var.getName(), true);
   }
   
   public MidiControl(int mapping, YoVariable<?> var, double max, double min, double exponent, double hires)
   {
      this(mapping, var, max, min, exponent, hires, var.getName(), true);
   }

   public MidiControl(int mapping, YoVariable<?> var, double max, double min, double exponent, double hires, String name)
   {
      this(mapping, var, max, min, exponent, hires, name, true);
   }

   public MidiControl(int mapping, YoVariable<?> var, double max, double min, double exponent, double hires, String name, boolean notify)
   {
      this.mapping = mapping;
      this.currentVal = var.getValueAsDouble();
      this.reset = this.currentVal;
      this.min = min;
      this.max = max;
      this.var = var;
      this.exponent = exponent;
      this.hires = hires;
      this.name = name;
   }
}
