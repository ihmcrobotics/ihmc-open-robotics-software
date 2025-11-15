package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTree;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Renders action execution status tracking UI elements.
 *
 * TODO: Add even more compressed representation. Radial plots.
 */
public class RDXActionProgressWidgetsManager
{
   public enum Type
   {
      /** Show only the estimated time remaining as a progress bar.. */
      TIME_ONLY,
      /** Show progress bars which are more compact than full plots. */
      PROGRESS_BARS,
      /** Show full side scrolling plots which show more information about execution but take up more space. */
      SCROLLING_PLOTS
   }
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final SortedSet<RDXActionNode<?, ?>> sortedActionNodesToRender = new TreeSet<>(Comparator.comparingInt(node -> node.getState().getLeafIndex()));
   private final ArrayList<RDXActionNode<?, ?>> actionNodesToRender = new ArrayList<>();
   private int emptyPlotIndex;
   private int numberOfLines;

   public RDXActionProgressWidgetsManager()
   {
      ImPlotTools.ensureImPlotInitialized();
   }

   public void render()
   {
      emptyPlotIndex = 0;
      numberOfLines = 0;

      actionNodesToRender.clear();
      actionNodesToRender.addAll(sortedActionNodesToRender);

      boolean containsFootsteps = false;
      boolean containsHandMovements = false;
      boolean containsSakeHandConfiguration = false;
      boolean containsAbilityHandConfiguration = false;
      for (RDXActionNode<?, ?> action : actionNodesToRender)
      {
         action.getProgressWidgets().update();

         if (action instanceof RDXFootstepPlanAction)
            containsFootsteps = true;
         if (action instanceof RDXHandPoseAction || action instanceof RDXScrewPrimitiveAction)
            containsHandMovements = true;
         if (action instanceof RDXSakeHandCommandAction)
            containsSakeHandConfiguration = true;
         if (action instanceof RDXAbilityHandAction)
            containsAbilityHandConfiguration = true;
      }
      boolean showPosePlots = containsFootsteps || containsHandMovements;

      ImGui.spacing();
      widgetAligner.text("Expected time remaining:");
      float dividedBarWidth = computeDividedBarWidth(); // Must be computed after above text
      handleRenderingBlankBar(false);
      for (int i = 0; i < actionNodesToRender.size(); i++)
      {
         actionNodesToRender.get(i).getProgressWidgets().renderElapsedTimeBar(dividedBarWidth);
         sameLineExceptLast(i);
      }
      ++numberOfLines;
      ImGui.spacing();

      boolean timeOnly = RDXBehaviorTree.SETTINGS.getProgressWidgetsType() == Type.TIME_ONLY;
      boolean renderAsPlots = RDXBehaviorTree.SETTINGS.getProgressWidgetsType() == Type.SCROLLING_PLOTS;
      if (!timeOnly)
      {
         if (containsFootsteps)
         {
            widgetAligner.text("Footstep completion:");
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               actionNodesToRender.get(i).getProgressWidgets().renderFootstepCompletion(dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();
         }

         if (showPosePlots)
         {
            widgetAligner.text("Position error (m):");
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               if (actionNodesToRender.get(i) instanceof RDXFootstepPlanAction)
                  actionNodesToRender.get(i).getProgressWidgets().renderFootPositions(dividedBarWidth, renderAsPlots);
               else
                  actionNodesToRender.get(i).getProgressWidgets().renderPositionError(dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();

            widgetAligner.text("Orientation error (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               if (actionNodesToRender.get(i) instanceof RDXFootstepPlanAction)
                  actionNodesToRender.get(i).getProgressWidgets().renderFootOrientations(dividedBarWidth, renderAsPlots);
               else
                  actionNodesToRender.get(i).getProgressWidgets().renderOrientationError(dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();
         }

         if (containsHandMovements)
         {
            widgetAligner.text("Hand force (N):");
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               actionNodesToRender.get(i).getProgressWidgets().renderHandForce(dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();

            widgetAligner.text("Hand torque (Nm):");
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               actionNodesToRender.get(i).getProgressWidgets().renderHandTorque(dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();
         }

         if (containsSakeHandConfiguration)
         {
            widgetAligner.text("Knuckle X1 (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               actionNodesToRender.get(i).getProgressWidgets().renderJointspacePositionError(0, dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();

            widgetAligner.text("Knuckle X2 (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
            handleRenderingBlankBar(true);
            for (int i = 0; i < actionNodesToRender.size(); i++)
            {
               actionNodesToRender.get(i).getProgressWidgets().renderJointspacePositionError(1, dividedBarWidth, renderAsPlots);
               sameLineExceptLast(i);
            }
            ++numberOfLines;
            ImGui.spacing();
         }

         if (containsAbilityHandConfiguration)
         {
            for (int f = 0; f < 6; f++)
            {
               widgetAligner.text("Hand %d (%s):".formatted(f, EuclidCoreMissingTools.DEGREE_SYMBOL));
               handleRenderingBlankBar(true);
               for (int i = 0; i < actionNodesToRender.size(); i++)
               {
                  actionNodesToRender.get(i).getProgressWidgets().renderJointspacePositionError(2, dividedBarWidth, renderAsPlots);
                  sameLineExceptLast(i);
               }
               ++numberOfLines;
               ImGui.spacing();
            }
         }
      }

      while (numberOfLines < (timeOnly ? 1 : 5))
      {
         widgetAligner.text("");
         renderBlankBar(true);
         ++numberOfLines;
         ImGui.spacing();
      }
   }

   private float computeDividedBarWidth()
   {
      // We compute the bar width to show them all together,
      // but we need to account for the spacing between them.
      float barWidthToSubtract = 0.0f;
      if (actionNodesToRender.size() > 1)
      {
         int barsPastOne = actionNodesToRender.size() - 1;
         float totalInnerSpacing = ImGui.getStyle().getItemSpacingX() * barsPastOne;
         barWidthToSubtract = totalInnerSpacing / actionNodesToRender.size();
      }
      return ImGui.getColumnWidth() / actionNodesToRender.size() - barWidthToSubtract;
   }

   private void sameLineExceptLast(int i)
   {
      if (i < actionNodesToRender.size() - 1)
         ImGui.sameLine();
   }

   private void handleRenderingBlankBar(boolean supportsPlots)
   {
      if (actionNodesToRender.isEmpty())
      {
         renderBlankBar(supportsPlots);
      }
   }

   private void renderBlankBar(boolean supportsPlots)
   {
      boolean renderAsPlots = RDXBehaviorTree.SETTINGS.getProgressWidgetsType() == Type.SCROLLING_PLOTS;
      RDXActionProgressWidgets.renderBlankProgress(labels.get("Empty Plot", emptyPlotIndex++), ImGui.getColumnWidth(), renderAsPlots, supportsPlots);
   }

   public SortedSet<RDXActionNode<?, ?>> getActionNodesToRender()
   {
      return sortedActionNodesToRender;
   }
}