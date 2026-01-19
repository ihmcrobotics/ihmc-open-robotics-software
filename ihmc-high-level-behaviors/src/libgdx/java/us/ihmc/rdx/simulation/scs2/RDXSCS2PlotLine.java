package us.ihmc.rdx.simulation.scs2;

import imgui.ImVec2;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.function.Consumer;

public class RDXSCS2PlotLine
{
   private final YoVariable yoVariable;
   private final Consumer<YoVariable> removeSelf;
   private final YoDouble yoDouble;
   LinkedYoVariable<YoDouble> linkedYoDoubleVariable;
   YoBufferPropertiesReadOnly bufferProperties;
   double[] data;
   final ArrayList<ImVec2> pointsList = new ArrayList<>();
   ImVec2[] points = new ImVec2[0];
   boolean isDragging = false;
   double minValue = Double.POSITIVE_INFINITY;
   double maxValue = Double.NEGATIVE_INFINITY;

   public RDXSCS2PlotLine(YoVariable yoVariable, Consumer<YoVariable> removeSelf)
   {
      this.yoVariable = yoVariable;
      this.removeSelf = removeSelf;
      this.yoDouble = yoVariable instanceof YoDouble ? (YoDouble) yoVariable : null;
   }

   public void setupLinkedVariable(RDXYoManager yoManager)
   {
      if (yoDouble != null && linkedYoDoubleVariable == null)
      {
         linkedYoDoubleVariable = (LinkedYoVariable) yoManager.newLinkedYoVariable(yoDouble);
         linkedYoDoubleVariable.addUser(this);
      }
   }

   public void updateData()
   {
      if (linkedYoDoubleVariable != null)
      {
         linkedYoDoubleVariable.pull();
         if (linkedYoDoubleVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample bufferSample = linkedYoDoubleVariable.pollRequestedBufferSample();
            bufferProperties = bufferSample.getBufferProperties();
            data = (double[]) bufferSample.getSample();
         }
         linkedYoDoubleVariable.requestEntireBuffer();
      }

      if (data == null)
         return;

      while (pointsList.size() < data.length)
         pointsList.add(new ImVec2());

      if (data.length != points.length)
      {
         points = new ImVec2[data.length];
         for (int i = 0; i < points.length; i++)
            points[i] = pointsList.get(i);
      }

      minValue = Double.POSITIVE_INFINITY;
      maxValue = Double.NEGATIVE_INFINITY;
      for (double datum : data)
      {
         minValue = Math.min(minValue, datum);
         maxValue = Math.max(maxValue, datum);
      }
   }

   public String getVariableName()
   {
      return yoVariable.getName();
   }

   public String getValueString(int bufferIndex)
   {
      return null;
   }
}
