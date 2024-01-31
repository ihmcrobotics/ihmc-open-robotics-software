package us.ihmc.rdx.imgui;

import imgui.ImVec2;
import imgui.extension.implot.ImPlot;
import imgui.extension.implot.ImPlotContext;
import imgui.extension.implot.ImPlotStyle;
import imgui.extension.implot.flag.ImPlotAxisFlags;
import imgui.extension.implot.flag.ImPlotFlags;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

public final class ImPlotTools
{
   public static final int IMPLOT_AUTO = -1;

   private static ImPlotContext context = null;

   private static ImVec2 emptyPlotSize = new ImVec2();

   public static void destroy()
   {
      if (context != null)
      {
         ImPlot.destroyContext(context);
         context = null;
      }
   }

   /**
    * We are assuming we don't ever need more than one context.
    */
   public static ImPlotContext ensureImPlotInitialized()
   {
      if (context == null)
      {
         context = ImPlot.createContext();
         ImPlot.setCurrentContext(context);
      }
      return context;
   }

   public static ImPlotContext getContext()
   {
      return context;
   }

   public static void setSCSStyle()
   {
      ImPlotStyle style = ImPlot.getStyle();
      style.setPlotPadding(new ImVec2(0, 0));
      style.setLabelPadding(new ImVec2(3, 0));
      style.setLegendPadding(new ImVec2(0, 0));
      style.setLegendInnerPadding(new ImVec2(5, 0));
      style.setAntiAliasedLines(true);
   }

   public static void renderEmptyPlotArea(String label, float width, float height)
   {
      emptyPlotSize.set(width, height);
      if (ImPlot.beginPlot(label, "", "", emptyPlotSize,
                           ImPlotFlags.NoMenus | ImPlotFlags.NoBoxSelect | ImPlotFlags.NoTitle | ImPlotFlags.NoMousePos,
                           ImPlotAxisFlags.NoDecorations | ImPlotAxisFlags.NoLabel,
                           ImPlotAxisFlags.NoDecorations | ImPlotAxisFlags.NoLabel))
      {
         ImPlot.endPlot();
      }
   }

   //Despite being identical code-wise, these need different methods because of casts from different primitive types
   public static Double[] convertArray(int[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(long[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(float[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(double[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = array[i];
      }
      return output;
   }

   public static double[] newNaNFilledBuffer(int bufferSize)
   {
      double[] buffer = new double[bufferSize];
      Arrays.fill(buffer, Double.NaN);
      return buffer;
   }

   public static double[] newZeroFilledBuffer(int bufferSize)
   {
      double[] buffer = new double[bufferSize];
      Arrays.fill(buffer, 0);
      return buffer;
   }

   public static <T extends Number> Integer[] createIndex(T[] array)
   {
      return createIndex(array, 0);
   }

   public static <T extends Number> Integer[] createIndex(T[] array, int start)
   {
      Integer[] output = new Integer[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = i + start;
      }

      return output;
   }

   public static double[] createIndex(int bufferSize)
   {
      double[] index = new double[bufferSize];
      for (int i = 0; i < bufferSize; i++)
      {
         index[i] = i;
      }

      return index;
   }

   /** @deprecated THIS METHOD IS SUPER EXPENSIVE, DON'T USE */
   public static Double[] removeNullElements(Double[] array)
   {
      return removeNullElements(array, Double.class);
   }

   /** @deprecated THIS METHOD IS SUPER EXPENSIVE, DON'T USE */
   public static <T extends Number> T[] removeNullElements(T[] array, Class<T> arrayClass)
   {
      if (Arrays.stream(array).noneMatch(Objects::isNull))
         return array;

      ArrayList<T> output = new ArrayList<>();
      for (T t : array)
      {
         if (t != null)
            output.add(t);
      }

      T[] store = (T[]) Array.newInstance(arrayClass, output.size());
      return output.toArray(store);
   }
}
