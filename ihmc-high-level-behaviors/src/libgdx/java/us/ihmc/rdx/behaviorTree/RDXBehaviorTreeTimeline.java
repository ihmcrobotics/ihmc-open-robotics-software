package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.rdx.behaviorTree.control.RDXActionSequence;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;

import com.badlogic.gdx.graphics.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class RDXBehaviorTreeTimeline extends RDXPanel
{
   private static final float SECONDS_TO_PIXELS = 80.0f;
   private static final float MIN_BAR_WIDTH_PIXELS = 12.0f;
   private static final float BAR_HEIGHT_SCALE = 0.8f;
   private static final float TEXT_PADDING_X = 4.0f;
   private static final double DEFAULT_NOMINAL_DURATION = 1.0;
   private static final float RULER_HEIGHT_SCALE = 1.2f;
   private static final float TICK_HEIGHT_SCALE = 0.5f;
   private static final float NOW_LINE_THICKNESS = 2.0f;

   private RDXBehaviorTreeRootNode rootNode;
   private final Map<Long, Integer> nodeIdToColor = new HashMap<>();
   private float lastHorizontalScrollX = 0.0f;

   public RDXBehaviorTreeTimeline()
   {
      super("Behavior Timeline");

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      if (rootNode == null)
      {
         ImGui.text("Root node is null. No scene.");
         return;
      }

      List<LeafNodeState<?>> leaves = rootNode.getState().getOrderedLeaves();
      if (leaves.isEmpty())
      {
         ImGui.text("No leaves to render.");
         return;
      }

      float rowHeight = ImGui.getFrameHeight();
      float rowSpacing = ImGui.getStyle().getItemSpacingY();
      float barHeight = rowHeight * BAR_HEIGHT_SCALE;
      float rulerHeight = rowHeight * RULER_HEIGHT_SCALE;

      double[] nominalStartTimes = new double[leaves.size()];
      double[] nominalDurations = new double[leaves.size()];

      for (int i = 0; i < leaves.size(); i++)
      {
         LeafNodeState<?> leaf = leaves.get(i);
         double duration = getDurationSeconds(leaf);
         nominalDurations[i] = duration;

         int executeAfterIndex = leaf.getExecuteAfterLeafIndex();
         double startTime = 0.0;
         if (executeAfterIndex >= 0 && executeAfterIndex < i)
            startTime = nominalStartTimes[executeAfterIndex] + nominalDurations[executeAfterIndex];

         nominalStartTimes[i] = startTime;
      }

      int nextExecutionIndex = rootNode.getState().getExecutionNextIndex();
      RDXLeafNode<?, ?> nextLeafNode = findLeafNodeByIndex(rootNode, nextExecutionIndex);
      RDXBehaviorTreeNode<?, ?> siblingsParent = nextLeafNode != null && nextLeafNode.getParent() != null ? nextLeafNode.getParent() : rootNode;
      RDXActionSequence currentActionSequence = findAncestorActionSequence(nextLeafNode);
      double timeOffset = 0.0;
      if (currentActionSequence != null)
      {
         TimeRange sequenceRange = new TimeRange();
         collectLeafTimeRange(currentActionSequence, nominalStartTimes, nominalDurations, sequenceRange);
         if (sequenceRange.hasAny())
            timeOffset = sequenceRange.minStart;
      }

      List<TimelineItem> items = new ArrayList<>();
      for (RDXBehaviorTreeNode<?, ?> sibling : siblingsParent.getChildren())
      {
         if (sibling instanceof RDXActionSequence)
         {
            TimelineItem sequenceItem = buildActionSequenceItem(sibling, nominalStartTimes, nominalDurations, timeOffset);
            if (sequenceItem != null)
               items.add(sequenceItem);
         }
         else
         {
            collectLeafItems(sibling, items, nominalStartTimes, nominalDurations, timeOffset);
         }
      }

      if (items.isEmpty())
      {
         ImGui.text("No timeline items to render.");
         return;
      }

      double displayMaxEndTime = 0.0;
      for (TimelineItem item : items)
         displayMaxEndTime = Math.max(displayMaxEndTime, item.startTime + item.duration);

      float totalHeight = items.size() * (rowHeight + rowSpacing);
      float totalWidth = (float) (displayMaxEndTime * SECONDS_TO_PIXELS);

      int textColor = ImGui.getColorU32(ImGuiCol.Text);
      int outlineColor = ImGui.getColorU32(ImGuiCol.TextDisabled);
      int nowLineColor = ImGui.getColorU32(ImGuiCol.SliderGrab);
      boolean hasNextExecution = nextExecutionIndex >= 0 && nextExecutionIndex < leaves.size();
      float nextExecutionTime = hasNextExecution ? (float) (nominalStartTimes[nextExecutionIndex] - timeOffset) : Float.NaN;

      // Time ruler (stays visible)
      ImGui.beginChild("BehaviorTimelineRuler", 0.0f, rulerHeight, false, ImGuiWindowFlags.NoScrollbar);
      {
         ImGui.setScrollX(lastHorizontalScrollX);
         if (ImGui.isWindowHovered() && ImGui.isMouseDragging(ImGuiMouseButton.Middle, 0.0f))
         {
            float scrollX = ImGui.getScrollX() - ImGui.getIO().getMouseDeltaX();
            ImGui.setScrollX(scrollX);
            lastHorizontalScrollX = ImGui.getScrollX();
         }
         float originX = ImGui.getCursorScreenPosX();
         float originY = ImGui.getCursorScreenPosY();
         float rulerBaselineY = originY + rulerHeight - ImGui.getStyle().getItemSpacingY();

         ImGui.getWindowDrawList().addLine(originX, rulerBaselineY, originX + totalWidth, rulerBaselineY, outlineColor, 1.0f);
         int secondsMax = (int) Math.ceil(displayMaxEndTime);
         for (int sec = 0; sec <= secondsMax; sec++)
         {
            float tickX = originX + sec * SECONDS_TO_PIXELS;
            float tickTopY = rulerBaselineY - rowHeight * TICK_HEIGHT_SCALE;
            ImGui.getWindowDrawList().addLine(tickX, tickTopY, tickX, rulerBaselineY, outlineColor, 1.0f);
            ImGui.getWindowDrawList().addText(tickX + 2.0f, tickTopY - ImGui.getFontSize(), textColor, sec + "s");
         }
         if (hasNextExecution)
         {
            float nowX = originX + nextExecutionTime * SECONDS_TO_PIXELS;
            ImGui.getWindowDrawList().addLine(nowX, originY, nowX, originY + rulerHeight, nowLineColor, NOW_LINE_THICKNESS);
         }

         ImGui.dummy(Math.max(totalWidth, ImGui.getContentRegionAvailX()), rulerHeight);
      }
      ImGui.endChild();

      // Bars (scrollable)
      ImGui.beginChild("BehaviorTimelineScroll",
                       0.0f,
                       ImGui.getContentRegionAvailY(),
                       false,
                       ImGuiWindowFlags.HorizontalScrollbar);
      {
         ImGui.setScrollX(lastHorizontalScrollX);
         if (ImGui.isWindowHovered() && ImGui.isMouseDragging(ImGuiMouseButton.Middle, 0.0f))
         {
            float scrollX = ImGui.getScrollX() - ImGui.getIO().getMouseDeltaX();
            float scrollY = ImGui.getScrollY() - ImGui.getIO().getMouseDeltaY();
            ImGui.setScrollX(scrollX);
            ImGui.setScrollY(scrollY);
         }
         float originX = ImGui.getCursorScreenPosX();
         float originY = ImGui.getCursorScreenPosY();

         if (hasNextExecution)
         {
            float nowX = originX + nextExecutionTime * SECONDS_TO_PIXELS;
            ImGui.getWindowDrawList().addLine(nowX, originY, nowX, originY + totalHeight, nowLineColor, NOW_LINE_THICKNESS);
         }

         for (int i = 0; i < items.size(); i++)
         {
            TimelineItem item = items.get(i);
            float rowTop = originY + i * (rowHeight + rowSpacing);
            float barStartX = originX + (float) (item.startTime * SECONDS_TO_PIXELS);
            float barWidth = Math.max(MIN_BAR_WIDTH_PIXELS, (float) (item.duration * SECONDS_TO_PIXELS));
            float barStartY = rowTop + (rowHeight - barHeight) * 0.5f;
            float barEndX = barStartX + barWidth;
            float barEndY = barStartY + barHeight;

            int barColor = getNodeColor(item.nodeId);
            ImGui.getWindowDrawList().addRectFilled(barStartX, barStartY, barEndX, barEndY, barColor);
            ImGui.getWindowDrawList().addRect(barStartX, barStartY, barEndX, barEndY, outlineColor);
            ImGui.getWindowDrawList().addText(barStartX + TEXT_PADDING_X,
                                              barStartY + (barHeight - ImGui.getFontSize()) * 0.5f,
                                              textColor,
                                              item.label);
         }

         ImGui.dummy(Math.max(totalWidth, ImGui.getContentRegionAvailX()), totalHeight);
         lastHorizontalScrollX = ImGui.getScrollX();
      }
      ImGui.endChild();
   }

   public static boolean renderIcon()
   {
      float height = ImGui.getFrameHeight();
      float scale = ImGui.getFontSize();
      float itemWidth = 2.0f * scale;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, height);


      int lineColor = ImGui.getColorU32(ImGuiCol.Text);


      float offsetX = ImGui.getCursorScreenPosX();
      float offsetY = ImGui.getCursorScreenPosY();

      if (isHovered)
      {
         ImGui.getWindowDrawList().addRectFilled(offsetX, offsetY, offsetX + 0.9f * scale, offsetY + 0.3f * scale, ImGuiTools.LIGHT_BLUE);
         ImGui.getWindowDrawList().addRectFilled(offsetX + 0.35f * scale, offsetY + 0.3f * scale,
                                           offsetX + (0.9f + 0.35f) * scale, offsetY + 2f * 0.3f * scale, ImGuiTools.DARK_GREEN);
         ImGui.getWindowDrawList().addRectFilled(offsetX + (2f * 0.35f) * scale, offsetY + (2f * 0.3f) * scale,
                                           offsetX + 1.5f * scale, offsetY + 3f * 0.3f * scale, ImGuiTools.YELLOW);
      }

      ImGui.getWindowDrawList().addRect(offsetX, offsetY, offsetX + 0.9f * scale, offsetY + 0.3f * scale, lineColor);
      ImGui.getWindowDrawList().addRect(offsetX + 0.35f * scale, offsetY + 0.3f * scale,
                                        offsetX + (0.9f + 0.35f) * scale, offsetY + 2f * 0.3f * scale, lineColor);
      ImGui.getWindowDrawList().addRect(offsetX + (2f * 0.35f) * scale, offsetY + (2f * 0.3f) * scale,
                                        offsetX + 1.5f * scale, offsetY + 3f * 0.3f * scale, lineColor);


      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   public void setRootNode(RDXBehaviorTreeRootNode rootNode)
   {
      this.rootNode = rootNode;
   }

   private double getDurationSeconds(LeafNodeState<?> leaf)
   {
      if (leaf instanceof ActionNodeState<?> action)
      {
         double elapsed = action.getElapsedExecutionTime();
         if (!Double.isNaN(elapsed) && elapsed > 0.0)
            return elapsed;

         double nominal = action.getNominalExecutionDuration();
         if (!Double.isNaN(nominal) && nominal > 0.0)
            return nominal;
      }

      return DEFAULT_NOMINAL_DURATION;
   }

   private int getNodeColor(long nodeId)
   {
      Integer cached = nodeIdToColor.get(nodeId);
      if (cached != null)
         return cached;

      Random seededRandom = new Random(nodeId);
      int r = (int) (seededRandom.nextDouble() * 256);
      int g = (int) (seededRandom.nextDouble() * 256);
      int b = (int) (seededRandom.nextDouble() * 256);
      int color = new Color(r / 255.0f, g / 255.0f, b / 255.0f, 100.0f / 255.0f).toIntBits();
      nodeIdToColor.put(nodeId, color);
      return color;
   }

   private RDXLeafNode<?, ?> findLeafNodeByIndex(RDXBehaviorTreeNode<?, ?> node, int leafIndex)
   {
      if (node instanceof RDXLeafNode<?, ?> leaf && leaf.getState().getLeafIndex() == leafIndex)
         return leaf;

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         RDXLeafNode<?, ?> found = findLeafNodeByIndex(child, leafIndex);
         if (found != null)
            return found;
      }

      return null;
   }

   private RDXActionSequence findAncestorActionSequence(RDXBehaviorTreeNode<?, ?> node)
   {
      RDXBehaviorTreeNode<?, ?> current = node;
      while (current != null)
      {
         if (current instanceof RDXActionSequence sequence)
            return sequence;
         current = current.getParent();
      }
      return null;
   }

   private void collectLeafItems(RDXBehaviorTreeNode<?, ?> node,
                                 List<TimelineItem> items,
                                 double[] nominalStartTimes,
                                 double[] nominalDurations,
                                 double timeOffset)
   {
      if (node instanceof RDXLeafNode<?, ?> leaf)
      {
         int leafIndex = leaf.getState().getLeafIndex();
         if (leafIndex >= 0 && leafIndex < nominalStartTimes.length)
         {
            items.add(new TimelineItem(leaf.getDefinition().getName(),
                                       nominalStartTimes[leafIndex] - timeOffset,
                                       nominalDurations[leafIndex],
                                       leaf.getState().getID()));
         }
         return;
      }

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         collectLeafItems(child, items, nominalStartTimes, nominalDurations, timeOffset);
   }

   private TimelineItem buildActionSequenceItem(RDXBehaviorTreeNode<?, ?> sequenceNode,
                                                double[] nominalStartTimes,
                                                double[] nominalDurations,
                                                double timeOffset)
   {
      TimeRange range = new TimeRange();
      collectLeafTimeRange(sequenceNode, nominalStartTimes, nominalDurations, range);
      if (!range.hasAny())
         return null;

      return new TimelineItem(sequenceNode.getDefinition().getName(),
                              range.minStart - timeOffset,
                              range.maxEnd - range.minStart,
                              sequenceNode.getState().getID());
   }

   private void collectLeafTimeRange(RDXBehaviorTreeNode<?, ?> node,
                                     double[] nominalStartTimes,
                                     double[] nominalDurations,
                                     TimeRange range)
   {
      if (node instanceof RDXLeafNode<?, ?> leaf)
      {
         int leafIndex = leaf.getState().getLeafIndex();
         if (leafIndex >= 0 && leafIndex < nominalStartTimes.length)
         {
            double start = nominalStartTimes[leafIndex];
            double end = start + nominalDurations[leafIndex];
            range.include(start, end);
         }
         return;
      }

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         collectLeafTimeRange(child, nominalStartTimes, nominalDurations, range);
   }

   private static class TimeRange
   {
      private double minStart = Double.POSITIVE_INFINITY;
      private double maxEnd = Double.NEGATIVE_INFINITY;

      private void include(double start, double end)
      {
         minStart = Math.min(minStart, start);
         maxEnd = Math.max(maxEnd, end);
      }

      private boolean hasAny()
      {
         return minStart <= maxEnd;
      }
   }

   private static class TimelineItem
   {
      private final String label;
      private final double startTime;
      private final double duration;
      private final long nodeId;

      private TimelineItem(String label, double startTime, double duration, long nodeId)
      {
         this.label = label;
         this.startTime = startTime;
         this.duration = duration;
         this.nodeId = nodeId;
      }
   }
}
