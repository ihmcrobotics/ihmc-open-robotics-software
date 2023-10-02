package us.ihmc.rdx.ui.behavior.editor;

import imgui.internal.ImGui;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXMultipleActionProgressBars
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;

   private final RecyclingArrayList<RDXSingleActionProgressBars> actionProgressBars = new RecyclingArrayList<>(RDXSingleActionProgressBars::new);
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();

   public void render()
   {
      ImGui.text("Executing: ");
      ImGui.sameLine();

      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         ImGui.text("%s (%s)".formatted(actionProgressBar.getAction().getState().getDefinition().getDescription(),
                                        actionProgressBar.getAction().getActionTypeTitle()));
      }

      widgetAligner.text("Expected time remaining:");

      // We compute the bar width to show them all together,
      // but we need to account for the spacing between them.
      float barWidthToSubtract = 0.0f;
      if (actionProgressBars.size() > 1)
      {
         int barsPastOne = actionProgressBars.size() - 1;
         float totalInnerSpacing = ImGui.getStyle().getItemSpacingX() * barsPastOne;
         barWidthToSubtract = totalInnerSpacing / actionProgressBars.size();
      }
      float dividedBarWidth = ImGui.getColumnWidth() / actionProgressBars.size() - barWidthToSubtract;

      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         double elapsedTime = actionProgressBar.getActionExecutionStatusMessage().getElapsedExecutionTime();
         double nominalDuration = actionProgressBar.getActionExecutionStatusMessage().getNominalExecutionDuration();
         double percentComplete = elapsedTime / nominalDuration;
         double percentLeft = 1.0 - percentComplete;
         ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, "%.2f / %.2f".formatted(elapsedTime, nominalDuration));
         ImGui.sameLine();
      }
      ImGui.spacing();
      widgetAligner.text("Position error (m):");
      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         double currentPositionError = actionProgressBar.getActionExecutionStatusMessage().getCurrentPositionDistanceToGoal();
         double startPositionError = actionProgressBar.getActionExecutionStatusMessage().getStartPositionDistanceToGoal();
         double positionTolerance = actionProgressBar.getActionExecutionStatusMessage().getPositionDistanceToGoalTolerance();
         double barEndValue = Math.max(Math.min(startPositionError, currentPositionError), 2.0 * positionTolerance);
         double toleranceMarkPercent = positionTolerance / barEndValue;
         int barColor = currentPositionError < positionTolerance ? ImGuiTools.GREEN : ImGuiTools.RED;
         double percentLeft = currentPositionError / barEndValue;
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                      dividedBarWidth,
                                      barColor,
                                      percentLeft,
                                      toleranceMarkPercent,
                                      "%.2f / %.2f".formatted(currentPositionError, startPositionError));
         ImGui.sameLine();
      }
      ImGui.spacing();
      widgetAligner.text("Orientation error (%s):".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL));
      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         double currentOrientationError = actionProgressBar.getActionExecutionStatusMessage().getCurrentOrientationDistanceToGoal();
         double startOrientationError = actionProgressBar.getActionExecutionStatusMessage().getStartOrientationDistanceToGoal();
         double orientationTolerance = actionProgressBar.getActionExecutionStatusMessage().getOrientationDistanceToGoalTolerance();
         double barEndValue = Math.max(Math.min(startOrientationError, currentOrientationError), 2.0 * orientationTolerance);
         double toleranceMarkPercent = orientationTolerance / barEndValue;
         int barColor = currentOrientationError < orientationTolerance ? ImGuiTools.GREEN : ImGuiTools.RED;
         double percentLeft = currentOrientationError / barEndValue;
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                      dividedBarWidth,
                                      barColor,
                                      percentLeft,
                                      toleranceMarkPercent,
                                      "%.2f / %.2f".formatted(Math.toDegrees(currentOrientationError), Math.toDegrees(startOrientationError)));
         ImGui.sameLine();
      }
      ImGui.spacing();
      widgetAligner.text("Footstep completion:");
      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         int incompleteFootsteps = actionProgressBar.getActionExecutionStatusMessage().getNumberOfIncompleteFootsteps();
         int totalFootsteps = actionProgressBar.getActionExecutionStatusMessage().getTotalNumberOfFootsteps();
         double percentLeft = incompleteFootsteps / (double) totalFootsteps;
         ImGui.progressBar((float) percentLeft, ImGui.getColumnWidth(), PROGRESS_BAR_HEIGHT, "%d / %d".formatted(incompleteFootsteps, totalFootsteps));
         ImGui.sameLine();
      }
      ImGui.spacing();
      widgetAligner.text("Hand wrench linear (N?):");
      for (RDXSingleActionProgressBars actionProgressBar : actionProgressBars)
      {
         double limit = 20.0;
         double force = actionProgressBar.getActionExecutionStatusMessage().getHandWrenchMagnitudeLinear();
         int barColor = force < limit ? ImGuiTools.GREEN : ImGuiTools.RED;
         ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT, dividedBarWidth, barColor, force / limit, 0.5, "%.2f".formatted(force));
         ImGui.sameLine();
      }
      ImGui.spacing();
   }

   public RecyclingArrayList<RDXSingleActionProgressBars> getActionProgressBars()
   {
      return actionProgressBars;
   }
}
