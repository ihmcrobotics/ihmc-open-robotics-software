package us.ihmc.rdx.ui.yo;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImDrawFlags;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.commons.MathTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.function.Consumer;

public class ImGuiSCSPlotLine
{
   private final YoVariable yoVariable;
   private final Consumer<YoVariable> removeSelf;
   private Session session;
   private final YoDouble yoDouble;
   private LinkedYoVariable<YoDouble> linkedYoDoubleVariable;
   private YoBufferPropertiesReadOnly bufferProperties;
   private double[] data;
   private final ArrayList<ImVec2> pointsList = new ArrayList<>();
   private ImVec2[] points = new ImVec2[0];
   private boolean isDragging = false;

   public ImGuiSCSPlotLine(YoVariable yoVariable, Consumer<YoVariable> removeSelf)
   {
      this.yoVariable = yoVariable;
      this.removeSelf = removeSelf;
      this.yoDouble = yoVariable instanceof YoDouble ? (YoDouble) yoVariable : null;
   }

   public void setupLinkedVariable(RDXYoManager yoManager)
   {
      if (yoDouble != null && linkedYoDoubleVariable == null)
      {
         session = yoManager.getSession();
         linkedYoDoubleVariable = (LinkedYoVariable) yoManager.newLinkedYoVariable(yoDouble);
         linkedYoDoubleVariable.addUser(this);
      }
   }

   public void render(float plotWidth, float plotHeight, int lineIndex) // TODO: Refactor out the code from this
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

      double minValue = Double.POSITIVE_INFINITY;
      double maxValue = Double.NEGATIVE_INFINITY;
      for (double datum : data)
      {
         minValue = Math.min(minValue, datum);
         maxValue = Math.max(maxValue, datum);
      }
      double range = maxValue - minValue;

      float cursorX = ImGui.getCursorScreenPosX();
      float cursorY = ImGui.getCursorScreenPosY();
      int fontSize = ImGui.getFontSize();

      float lineAreaHeight = plotHeight - 1.1f * fontSize;
      for (int i = 0; i < points.length; i++)
      {
         float x = cursorX + i * plotWidth / data.length;
         double normalized = (data[i] - minValue) / range;
         float y = cursorY + lineAreaHeight * (1.0f - (float) normalized);
         points[i].set(x, y);
      }

      float plotMinX = cursorX;
      float plotMaxX = cursorX + plotWidth;
      float plotMinY = cursorY;
      float plotMaxY = cursorY + plotHeight;
      float mouseX = ImGui.getMousePosX();
      float mouseY = ImGui.getMousePosY();
      boolean mouseInPlotBounds = mouseX >= plotMinX && mouseX <= plotMaxX && mouseY >= plotMinY && mouseY <= plotMaxY;

      if (mouseInPlotBounds && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         isDragging = true;
      if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
         isDragging = false;

      if (isDragging)
      {
         float dragXPlot = (float) MathTools.clamp(ImGui.getMousePosX(), cursorX, cursorX + plotWidth);
         session.submitBufferIndexRequest(Math.round((dragXPlot - cursorX) * data.length / plotWidth));
      }

      int currentIndex = bufferProperties.getCurrentIndex();
      float verticalLineX = cursorX + currentIndex * plotWidth / data.length;
      ImGui.getWindowDrawList().addLine(verticalLineX, cursorY, verticalLineX, cursorY + lineAreaHeight, ImGuiTools.BLACK);

      int numPoints;
      int color = ImGuiSCSPlot.CHART_COLORS[lineIndex % ImGuiSCSPlot.CHART_COLORS.length];
      int imDrawFlags = ImDrawFlags.None;
      float thickness = 1.0f;
      ImGui.getWindowDrawList().addPolyline(points, points.length, color, imDrawFlags, thickness);

      ImGui.getWindowDrawList().addText(cursorX + 0.05f * fontSize,
                                        cursorY + plotHeight - (lineIndex + 1.0f + 0.05f) * fontSize,
                                        color,
                                        linkedYoDoubleVariable.getLinkedYoVariable().getName() + " %.5f".formatted(data[currentIndex]));
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
