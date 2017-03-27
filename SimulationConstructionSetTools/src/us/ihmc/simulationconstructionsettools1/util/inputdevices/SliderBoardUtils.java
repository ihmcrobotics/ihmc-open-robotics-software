package us.ihmc.simulationconstructionsettools1.util.inputdevices;

import us.ihmc.robotics.MathTools;


public class SliderBoardUtils
{
   private SliderBoardUtils()
   {
      // disallow construction
   }

   public static double valueRatioConvertToDoubleWithExponents(MidiControl control, int sliderValue, int sliderBoardMax)
   {
      double max = control.max;
      double min = control.min;
      double exponent = control.exponent;
      double hires = control.hires;
      
      return valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
   }

   public static int valueRatioConvertToIntWithExponents(MidiControl control, int sliderBoardMax)
   {
      double max = control.max;
      double min = control.min;
      double exponent = control.exponent;
      double hires = control.hires;

      double value = 0;
      
      double minValueAllowed = Math.min(control.min, control.max);
      double maxValueAllowed = Math.max(control.min, control.max);
     
      value = MathTools.clamp(control.var.getValueAsDouble(), minValueAllowed, maxValueAllowed);

      return valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
   }

   public static double valueRatioConvertToDoubleWithExponents(int sliderValue, int sliderBoardMax, double max, double min, double exponent, double hires)
   {
      double sliderBoardMin = 0.0;

      double sliderBoardDelta = (double)(sliderBoardMax - sliderBoardMin);
      double percentHires = (hires - min) / (max - min);
      double sliderHires = sliderBoardMin + percentHires * sliderBoardDelta;

      double alpha;
      if (sliderValue >= sliderHires)
      {
         alpha = (((double) sliderValue) - sliderHires)/(sliderBoardMax - sliderHires);
      }
      else
      {
         alpha = (((double) sliderValue) - sliderHires)/(sliderHires - sliderBoardMin);
      }

      double beta = Math.pow(Math.abs(alpha), exponent);
      if (alpha < 0.0)
         beta = -beta;
      double value;

      if (alpha >= 0.0)
      {
         value = hires + beta * (max - hires); 
      }
      else
      {
         value = hires + beta * (hires - min); 
      }

      return value;
   }

   public static int valueRatioConvertToIntWithExponents(int sliderBoardMax, double max, double min, double exponent, double hires, double value)
   {
      double sliderBoardMin = 0.0;

      double sliderBoardDelta = (double)(sliderBoardMax - sliderBoardMin);
      double percentHires = (hires - min) / (max - min);
      double sliderHires = sliderBoardMin + percentHires * sliderBoardDelta;

      double beta;
      if (value >= hires)
      {
//         value = hires + beta * (max - hires); 
         beta = (value - hires) / (max - hires);
      }
      else
      {
//         value = hires + beta * (hires - min); 
         beta = (value - hires) / (hires - min);
      }

      double alpha = Math.pow(Math.abs(beta), 1.0 / exponent);

      if (beta < 0.0)
         alpha = -alpha;
      
      int sliderValue;
      if (alpha > 0.0)
      {
//       alpha = (((double) sliderValue) - sliderHires)/(sliderBoardMax - sliderHires);
         sliderValue = (int) Math.round(alpha * (sliderBoardMax - sliderHires) + sliderHires);
      }
      else
      {
//       alpha = (((double) sliderValue) - sliderHires)/(sliderHires - sliderBoardMin);
         sliderValue = (int) Math.round(alpha * (sliderHires - sliderBoardMin) + sliderHires);
      }
      
      
      return sliderValue;
   }
   
   public static double valueRatioConvertToDoubleWithExponents(int sliderValue, int sliderBoardMax, double max, double min, double exponent)
   {
      double hires = (max + min)/2.0;
      return valueRatioConvertToDoubleWithExponents(sliderValue, sliderBoardMax, max, min, exponent, hires);
   }

   public static int valueRatioConvertToIntWithExponents(int sliderBoardMax, double max, double min, double exponent, double value)
   {
      double hires = (max + min)/2.0;
      return valueRatioConvertToIntWithExponents(sliderBoardMax, max, min, exponent, hires, value);
   }

}
