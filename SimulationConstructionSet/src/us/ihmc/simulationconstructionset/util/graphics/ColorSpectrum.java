package us.ihmc.simulationconstructionset.util.graphics;

import java.awt.Color;

import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

/**
 * <p>Title: ColorSpectrum </p>
 *
 * <p>Description: Provides a mapping from scalar values to colors.
 * Colors are given as an array of RGB values.
 *
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author Brian Bonnlander
 * @version 1.0
 */
public class ColorSpectrum
{
   private static final double EPSILON = 10e-10;

   // private double minValue;
   // private double maxValue;
   float[][] RGB;
   Color[] colors;

   // RGB values should be between 0.0 and 1.0.
   public ColorSpectrum(float[][] RGB)
   {
      int numColors = RGB.length;
      this.RGB = RGB;
      this.colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = new Color(RGB[i][0], RGB[i][1], RGB[i][2]);
      }
   }

   // RGB values should be between 0.0 and 1.0.
   public ColorSpectrum(double[][] RGB)
   {
      int numColors = RGB.length;
      this.RGB = new float[RGB.length][3];
      this.colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         this.RGB[i][0] = (float) RGB[i][0];
         this.RGB[i][1] = (float) RGB[i][1];
         this.RGB[i][2] = (float) RGB[i][2];
         colors[i] = new Color(this.RGB[i][0], this.RGB[i][1], this.RGB[i][2]);
      }
   }

// public ColorSpectrum(Color[] colors, int intermediateColorsToGenerate)
// {
//    int numColors = intermediateColorsToGenerate;
////     Color[] intermediateColors = new Color[intermediateColorsToGenerate];
//    ArrayList<Color> intermediateColors = new ArrayList<Color>();
//    for (int i = 0; i < intermediateColorsToGenerate; i++)
//    {
//       intermediateColors.add(new Color(0.0f, 0.0f, 0.0f));
//    }
//    ArrayList<ArrayList> arrayOfArrays = JohnsUsefulFunctions.splitArrayIntoEqualishParts(intermediateColors, colors.length - 1);
//
//    int totalIndex = 0;
//    for(int arrayIndex = 0; arrayIndex < arrayOfArrays.size(); arrayIndex++)
//    {
//       for (int indexOfInnerArray = 0; indexOfInnerArray < arrayOfArrays.get(arrayIndex).size(); indexOfInnerArray++)
//       {
//          Color lastColor = colors[arrayIndex];
//          Color nextColor = colors[arrayIndex + 1];
//          double alpha = ((double) indexOfInnerArray) / ((double) arrayOfArrays.get(arrayIndex).size());
//          float r = (float) (lastColor.getRed() + (alpha * (nextColor.getRed() - lastColor.getRed()))) / 256.0f;
//          float g = (float) (lastColor.getGreen() + (alpha * (nextColor.getGreen() - lastColor.getGreen()))) / 256.0f;
//          float b = (float) (lastColor.getBlue() + (alpha * (nextColor.getBlue() - lastColor.getBlue()))) / 256.0f;
//
//          System.err.println("ColorSpectrum::ColorSpectrum : alpha " + alpha);
//          System.err.println("ColorSpectrum::ColorSpectrum : r " + r);
//          System.err.println("ColorSpectrum::ColorSpectrum : g " + g);
//          System.err.println("ColorSpectrum::ColorSpectrum : b " + b);
//
//          Color thisMorphedColor = new Color(r, g, b);
//          intermediateColors.set(totalIndex, thisMorphedColor);
//          totalIndex++;
//       }
//    }
//    this.colors = new Color[intermediateColors.size()];
//    intermediateColors.toArray(this.colors);
//    this.RGB = new float[numColors][3];
//    for (int i = 0; i < this.colors.length; i++)
//    {
//       this.colors[i].getColorComponents(RGB[i]);
//    }
// }


   public ColorSpectrum(Color[] colors)
   {
      int numColors = colors.length;
      this.colors = colors;
      this.RGB = new float[numColors][3];

      for (int i = 0; i < numColors; i++)
      {
         colors[i].getColorComponents(RGB[i]);
      }
   }

   public int size()
   {
      return colors.length;
   }

   public float[][] getRGBCopy()
   {
      float[][] RGBCopy = new float[RGB.length][3];
      for (int i = 0; i < RGB.length; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            RGBCopy[i][j] = RGB[i][j];
         }
      }

      return RGBCopy;
   }

   // Use the value to index pick one of the colors in the spectrum;
   // provide the relative scale.
   public Color getColor(double value, double minValue, double maxValue)
   {
      // Place value within range.
      value = Math.max(value, minValue);
      value = Math.min(value, maxValue);
      int numColors = colors.length;
      int index = (int) (numColors * (value - minValue) / (maxValue - minValue + EPSILON));

      if (index < 0)
         index = 0;
      if (index >= numColors)
         index = numColors - 1;

      return colors[index];
   }

   // Find the color index for the given value along the real line
   // and interpolate between two neighboring colors using the fractional part of the index.
   public Color getColorContinuous(double value, double minValue, double maxValue)
   {
      // Place value within range.
      value = Math.max(value, minValue);
      value = Math.min(value, maxValue);

      int numColors = colors.length;
      float index = (float) (numColors * (value - minValue) / (maxValue - minValue + EPSILON));
      int baseIndex = (int) index;

      // We can't interpolate if the color index is the final one.
      if (baseIndex >= numColors - 1)
         return colors[numColors - 1];


      float fractionalIndex = index - baseIndex;
      float[] RGBMix = new float[3];

      for (int i = 0; i < 3; i++)
      {
         RGBMix[i] = (float) (RGB[baseIndex][i] * (1.0 - fractionalIndex) + RGB[baseIndex + 1][i] * (fractionalIndex));
      }

      return new Color(RGBMix[0], RGBMix[1], RGBMix[2]);
   }

   public static ColorSpectrum jetColors(int numColors)
   {
      return jetColors(numColors, false);
   }

   public static ColorSpectrum jetColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 0, 0.6250}, {0, 0, 0.7500}, {0, 0, 0.8750}, {0, 0, 1.0000}, {0, 0.1250, 1.0000}, {0, 0.2500, 1.0000}, {0, 0.3750, 1.0000}, {0, 0.5000, 1.0000},
         {0, 0.6250, 1.0000}, {0, 0.7500, 1.0000}, {0, 0.8750, 1.0000}, {0, 1.0000, 1.0000}, {0.1250, 1.0000, 0.8750}, {0.2500, 1.0000, 0.7500},
         {0.3750, 1.0000, 0.6250}, {0.5000, 1.0000, 0.5000}, {0.6250, 1.0000, 0.3750}, {0.7500, 1.0000, 0.2500}, {0.8750, 1.0000, 0.1250}, {1.0000, 1.0000, 0},
         {1.0000, 0.8750, 0}, {1.0000, 0.7500, 0}, {1.0000, 0.6250, 0}, {1.0000, 0.5000, 0}, {1.0000, 0.3750, 0}, {1.0000, 0.2500, 0}, {1.0000, 0.1250, 0},
         {1.0000, 0, 0}, {0.8750, 0, 0}, {0.7500, 0, 0}, {0.6250, 0, 0}, {0.5000, 0, 0}
      };

      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];
      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }


   public static ColorSpectrum jet2Colors(int numColors)
   {
      return jetColors(numColors, false);
   }

   public static ColorSpectrum jet2Colors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 0, 1.0000},    // From end of HSV
         {0, 0.2000, 1.0000}, {0, 0.4375, 1.0000}, {0, 0.6250, 1.0000}, {0, 0.8125, 1.0000}, {0, 1.0000, 1.0000}, {0, 1.0000, 0.8125}, {0, 1.0000, 0.6250},
         {0, 1.0000, 0.4375}, {0, 1.0000, 0.2500}, {0, 1.0000, 0.0625}, {0.1250, 1.0000, 0}, {0.3750, 1.0000, 0},    // From Jet
         {0.5000, 1.0000, 0}, {0.6250, 1.0000, 0}, {0.7500, 1.0000, 0}, {0.8750, 1.0000, 0}, {1.0000, 1.0000, 0}, {1.0000, 0.8750, 0}, {1.0000, 0.7500, 0},
         {1.0000, 0.6250, 0}, {1.0000, 0.5000, 0}, {1.0000, 0.3750, 0}, {1.0000, 0.2500, 0}, {1.0000, 0.1250, 0}, {1.0000, 0, 0}, {0.8750, 0, 0},
         {0.7500, 0, 0}, {0.6250, 0, 0}, {0.5000, 0, 0}
      };

      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];
      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   public static ColorSpectrum hsvColors(int numColors)
   {
      return hsvColors(numColors, false);
   }

   public static ColorSpectrum hsvColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {1.0000, 0, 0}, {1.0000, 0.1875, 0}, {1.0000, 0.3750, 0}, {1.0000, 0.5625, 0}, {1.0000, 0.7500, 0}, {1.0000, 0.9375, 0}, {0.8750, 1.0000, 0},
         {0.6875, 1.0000, 0}, {0.5000, 1.0000, 0}, {0.3125, 1.0000, 0}, {0.1250, 1.0000, 0}, {0, 1.0000, 0.0625}, {0, 1.0000, 0.2500}, {0, 1.0000, 0.4375},
         {0, 1.0000, 0.6250}, {0, 1.0000, 0.8125}, {0, 1.0000, 1.0000}, {0, 0.8125, 1.0000}, {0, 0.6250, 1.0000}, {0, 0.4375, 1.0000}, {0, 0.2500, 1.0000},
         {0, 0.0625, 1.0000}, {0, 0, 1.0000}    /*
                                                 *  Removed the last part of HSV since both ends are red.
                                                 * {0.3125, 0, 1.0000},
                                                 * {0.5000, 0, 1.0000},
                                                 * {0.6875, 0, 1.0000},
                                                 * {0.8750, 0, 1.0000},
                                                 * {1.0000, 0, 0.9375},
                                                 * {1.0000, 0, 0.7500},
                                                 * {1.0000, 0, 0.5625},
                                                 * {1.0000, 0, 0.3750},
                                                 * {1.0000, 0, 0.1875}
                                                 */
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      // By default, reverse the original HSV map to match Jet better.
      if (reverseMap)
      {
         return new ColorSpectrum(colors);
      }

      return new ColorSpectrum(reverseMap(colors));
   }

   public static ColorSpectrum boneColors(int numColors)
   {
      return boneColors(numColors, false);
   }

   public static ColorSpectrum boneColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 0, 0.0104}, {0.0282, 0.0282, 0.0491}, {0.0565, 0.0565, 0.0877}, {0.0847, 0.0847, 0.1263}, {0.1129, 0.1129, 0.1650}, {0.1411, 0.1411, 0.2036},
         {0.1694, 0.1694, 0.2423}, {0.1976, 0.1976, 0.2809}, {0.2258, 0.2258, 0.3196}, {0.2540, 0.2540, 0.3582}, {0.2823, 0.2823, 0.3968},
         {0.3105, 0.3105, 0.4355}, {0.3387, 0.3491, 0.4637}, {0.3669, 0.3878, 0.4919}, {0.3952, 0.4264, 0.5202}, {0.4234, 0.4651, 0.5484},
         {0.4516, 0.5037, 0.5766}, {0.4798, 0.5423, 0.6048}, {0.5081, 0.5810, 0.6331}, {0.5363, 0.6196, 0.6613}, {0.5645, 0.6583, 0.6895},
         {0.5927, 0.6969, 0.7177}, {0.6210, 0.7356, 0.7460}, {0.6492, 0.7742, 0.7742}, {0.6930, 0.8024, 0.8024}, {0.7369, 0.8306, 0.8306},
         {0.7807, 0.8589, 0.8589}, {0.8246, 0.8871, 0.8871}, {0.8684, 0.9153, 0.9153}, {0.9123, 0.9435, 0.9435}, {0.9561, 0.9718, 0.9718},
         {1.0000, 1.0000, 1.0000}
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   public static ColorSpectrum copperColors(int numColors)
   {
      return copperColors(numColors, false);
   }

   public static ColorSpectrum copperColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 0, 0}, {0.0403, 0.0252, 0.0160}, {0.0806, 0.0504, 0.0321}, {0.1210, 0.0756, 0.0481}, {0.1613, 0.1008, 0.0642}, {0.2016, 0.1260, 0.0802},
         {0.2419, 0.1512, 0.0963}, {0.2823, 0.1764, 0.1123}, {0.3226, 0.2016, 0.1284}, {0.3629, 0.2268, 0.1444}, {0.4032, 0.2520, 0.1605},
         {0.4435, 0.2772, 0.1765}, {0.4839, 0.3024, 0.1926}, {0.5242, 0.3276, 0.2086}, {0.5645, 0.3528, 0.2247}, {0.6048, 0.3780, 0.2407},
         {0.6452, 0.4032, 0.2568}, {0.6855, 0.4284, 0.2728}, {0.7258, 0.4536, 0.2889}, {0.7661, 0.4788, 0.3049}, {0.8065, 0.5040, 0.3210},
         {0.8468, 0.5292, 0.3370}, {0.8871, 0.5544, 0.3531}, {0.9274, 0.5796, 0.3691}, {0.9677, 0.6048, 0.3852}, {1.0000, 0.6300, 0.4012},
         {1.0000, 0.6552, 0.4173}, {1.0000, 0.6804, 0.4333}, {1.0000, 0.7056, 0.4494}, {1.0000, 0.7308, 0.4654}, {1.0000, 0.7560, 0.4815},
         {1.0000, 0.7812, 0.4975}
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   public static ColorSpectrum hotColors(int numColors)
   {
      return hotColors(numColors, false);
   }

   public static ColorSpectrum hotColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0.0833, 0, 0}, {0.1667, 0, 0}, {0.2500, 0, 0}, {0.3333, 0, 0}, {0.4167, 0, 0}, {0.5000, 0, 0}, {0.5833, 0, 0}, {0.6667, 0, 0}, {0.7500, 0, 0},
         {0.8333, 0, 0}, {0.9167, 0, 0}, {1.0000, 0, 0}, {1.0000, 0.0833, 0}, {1.0000, 0.1667, 0}, {1.0000, 0.2500, 0}, {1.0000, 0.3333, 0},
         {1.0000, 0.4167, 0}, {1.0000, 0.5000, 0}, {1.0000, 0.5833, 0}, {1.0000, 0.6667, 0}, {1.0000, 0.7500, 0}, {1.0000, 0.8333, 0}, {1.0000, 0.9167, 0},
         {1.0000, 1.0000, 0}, {1.0000, 1.0000, 0.1250}, {1.0000, 1.0000, 0.2500}, {1.0000, 1.0000, 0.3750}, {1.0000, 1.0000, 0.5000}, {1.0000, 1.0000, 0.6250},
         {1.0000, 1.0000, 0.7500}, {1.0000, 1.0000, 0.8750}, {1.0000, 1.0000, 1.0000}
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   public static ColorSpectrum coolColors(int numColors)
   {
      return coolColors(numColors, false);
   }

   public static ColorSpectrum coolColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 1.0000, 1.0000}, {0.0323, 0.9677, 1.0000}, {0.0645, 0.9355, 1.0000}, {0.0968, 0.9032, 1.0000}, {0.1290, 0.8710, 1.0000}, {0.1613, 0.8387, 1.0000},
         {0.1935, 0.8065, 1.0000}, {0.2258, 0.7742, 1.0000}, {0.2581, 0.7419, 1.0000}, {0.2903, 0.7097, 1.0000}, {0.3226, 0.6774, 1.0000},
         {0.3548, 0.6452, 1.0000}, {0.3871, 0.6129, 1.0000}, {0.4194, 0.5806, 1.0000}, {0.4516, 0.5484, 1.0000}, {0.4839, 0.5161, 1.0000},
         {0.5161, 0.4839, 1.0000}, {0.5484, 0.4516, 1.0000}, {0.5806, 0.4194, 1.0000}, {0.6129, 0.3871, 1.0000}, {0.6452, 0.3548, 1.0000},
         {0.6774, 0.3226, 1.0000}, {0.7097, 0.2903, 1.0000}, {0.7419, 0.2581, 1.0000}, {0.7742, 0.2258, 1.0000}, {0.8065, 0.1935, 1.0000},
         {0.8387, 0.1613, 1.0000}, {0.8710, 0.1290, 1.0000}, {0.9032, 0.0968, 1.0000}, {0.9355, 0.0645, 1.0000}, {0.9677, 0.0323, 1.0000}, {1.0000, 0, 1.0000}
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   public static ColorSpectrum grayColors(int numColors)
   {
      return grayColors(numColors, false);
   }

   public static ColorSpectrum grayColors(int numColors, boolean reverseMap)
   {
      double[][] seedRGB = new double[][]
      {
         {0, 0, 0}, {0.0323, 0.0323, 0.0323}, {0.0645, 0.0645, 0.0645}, {0.0968, 0.0968, 0.0968}, {0.1290, 0.1290, 0.1290}, {0.1613, 0.1613, 0.1613},
         {0.1935, 0.1935, 0.1935}, {0.2258, 0.2258, 0.2258}, {0.2581, 0.2581, 0.2581}, {0.2903, 0.2903, 0.2903}, {0.3226, 0.3226, 0.3226},
         {0.3548, 0.3548, 0.3548}, {0.3871, 0.3871, 0.3871}, {0.4194, 0.4194, 0.4194}, {0.4516, 0.4516, 0.4516}, {0.4839, 0.4839, 0.4839},
         {0.5161, 0.5161, 0.5161}, {0.5484, 0.5484, 0.5484}, {0.5806, 0.5806, 0.5806}, {0.6129, 0.6129, 0.6129}, {0.6452, 0.6452, 0.6452},
         {0.6774, 0.6774, 0.6774}, {0.7097, 0.7097, 0.7097}, {0.7419, 0.7419, 0.7419}, {0.7742, 0.7742, 0.7742}, {0.8065, 0.8065, 0.8065},
         {0.8387, 0.8387, 0.8387}, {0.8710, 0.8710, 0.8710}, {0.9032, 0.9032, 0.9032}, {0.9355, 0.9355, 0.9355}, {0.9677, 0.9677, 0.9677},
         {1.0000, 1.0000, 1.0000}
      };
      ColorSpectrum seedSpectrum = new ColorSpectrum(seedRGB);
      Color[] colors = new Color[numColors];

      for (int i = 0; i < numColors; i++)
      {
         colors[i] = seedSpectrum.getColorContinuous(i, 0, numColors);
      }

      if (reverseMap)
      {
         return new ColorSpectrum(reverseMap(colors));
      }

      return new ColorSpectrum(colors);
   }

   private static Color[] reverseMap(Color[] colors)
   {
      int numColors = colors.length;
      Color[] reverse = new Color[numColors];
      for (int i = 0; i < numColors; i++)
      {
         reverse[i] = colors[numColors - i - 1];
      }

      return reverse;
   }

   public static void addCube(double cubeX, double cubeY, double cubeSize, Color color, SimulationConstructionSet scs)
   {
      final double CUBE_HEIGHT = 0.001;
      // Make cubes a tiny bit smaller so they don't blend together.
      cubeSize *= 0.9;

      
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(new Vector3d(cubeX, cubeY ,CUBE_HEIGHT));
      linkGraphics.addCube(cubeSize, cubeSize, cubeSize, new YoAppearanceRGBColor(new Color3f(color), 0.0));
      
      scs.addStaticLinkGraphics(linkGraphics);
   }

   public static void printSpectrum(ColorSpectrum spectrum, double cubeY, double cubeSize, SimulationConstructionSet scs)
   {
      int numColors = spectrum.size();
      double cubeX = -1.5;
      for (int j = 0; j < numColors; j++)
      {
         Color color = spectrum.getColor(j, 0, numColors);
         addCube(cubeX, cubeY, cubeSize, color, scs);
         cubeX += cubeSize;
      }
   }


   /* Print out the various spectra on SCS. */
   public static void main(String[] args)
   {
      // A spectrum size larger than 32 tests color interpolation.
      int spectrumSize = 100;

      double cubeSize = 0.03;
      double cubeY = 0.3;    // 0.5; for reverse maps

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("NullBot"), parameters);

      // Print all maps and print their reverse maps.
      boolean reverseMap = true;

      // for (int i = 0; i <= 1; i++)
      {
         reverseMap = !reverseMap;

         // jet
         cubeY -= 3 * cubeSize;
         ColorSpectrum spectrum = ColorSpectrum.jetColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // jet2
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.jet2Colors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // hsv
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.hsvColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // bone
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.boneColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // copper
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.copperColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // hot
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.hotColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // cool
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.coolColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);

         // gray
         cubeY -= 3 * cubeSize;
         spectrum = ColorSpectrum.grayColors(spectrumSize, reverseMap);
         printSpectrum(spectrum, cubeY, cubeSize, scs);
      }

      Thread thread = new Thread(scs);
      thread.start();
   }
}
