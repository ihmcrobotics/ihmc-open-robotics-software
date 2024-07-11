package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXPsyonicAbilityHandCommandAction;
import us.ihmc.rdx.ui.behavior.actions.RDXSakeHandCommandAction;
import us.ihmc.rdx.ui.behavior.actions.RDXScrewPrimitiveAction;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;

public class RDXActionProgressWidgetsManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
   private final SortedSet<RDXActionNode<?, ?>> sortedActionNodesToRender = new TreeSet<>(Comparator.comparingInt(node -> node.getState().getActionIndex()));
   private final ArrayList<RDXActionNode<?, ?>> actionNodesToRender = new ArrayList<>();
   private boolean renderAsPlots = true;
   private int emptyPlotIndex;
   private int numberOfLines;

   public void render()
   {
      emptyPlotIndex = 0;
      numberOfLines = 0;

      actionNodesToRender.clear();
      actionNodesToRender.addAll(sortedActionNodesToRender);

      boolean containsFootsteps = false;
      boolean containsHandMovements = false;
      boolean containsHandConfiguration = false;
      for (RDXActionNode<?, ?> action : actionNodesToRender)
      {
         action.getProgressWidgets().update();

         if (action instanceof RDXFootstepPlanAction)
            containsFootsteps = true;
         if (action instanceof RDXHandPoseAction || action instanceof RDXScrewPrimitiveAction)
            containsHandMovements = true;
         if (action instanceof RDXSakeHandCommandAction)
            containsHandConfiguration = true;
         if (action instanceof RDXPsyonicAbilityHandCommandAction)
            containsHandConfiguration = true;
      }
      boolean showPosePlots = containsFootsteps || containsHandMovements;

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

      if (containsHandConfiguration)
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

      while (numberOfLines < 5)
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
      RDXActionProgressWidgets.renderBlankProgress(labels.get("Empty Plot", emptyPlotIndex++), ImGui.getColumnWidth(), renderAsPlots, supportsPlots);
   }

   public boolean getRenderAsPlots()
   {
      return renderAsPlots;
   }

   public void setRenderAsPlots(boolean renderAsPlots)
   {
      this.renderAsPlots = renderAsPlots;
   }

   public SortedSet<RDXActionNode<?, ?>> getActionNodesToRender()
   {
      return sortedActionNodesToRender;
   }
}