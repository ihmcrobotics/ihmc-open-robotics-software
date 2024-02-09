package us.ihmc.rdx.ui.behavior.sequence;

import imgui.extension.implot.ImPlot;
import imgui.extension.implot.flag.ImPlotFlags;
import imgui.flag.ImGuiCond;
import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.communication.crdt.CRDTUnidirectionalVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.actions.RDXFootstepPlanAction;
import us.ihmc.rdx.ui.behavior.actions.RDXHandPoseAction;
import us.ihmc.rdx.ui.behavior.actions.RDXScrewPrimitiveAction;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RDXActionProgressWidgets
{
   public static final float PROGRESS_BAR_HEIGHT = 18.0f;
   public static final float PLOT_HEIGHT = 40.0f;

   private final RDXActionNode<?, ?> action;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImPlotPlot positionErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentPositionErrorPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotBasicDoublePlotLine desiredPositionErrorPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot orientationErrorPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine currentOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotBasicDoublePlotLine desiredOrientationPlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot[] jointspacePositionErrorPlot = new ImPlotPlot[ActionNodeState.SUPPORTED_NUMBER_OF_JOINTS];
   private final ImPlotBasicDoublePlotLine[] currentJointspacePositionErrorPlotLine = new ImPlotBasicDoublePlotLine[jointspacePositionErrorPlot.length];
   private final ImPlotBasicDoublePlotLine[] desiredJointspacePositionErrorPlotLine = new ImPlotBasicDoublePlotLine[jointspacePositionErrorPlot.length];
   private final ImPlotPlot footstepsRemainingPlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine footstepsRemainingPlotLine = new ImPlotBasicDoublePlotLine();
   private final SideDependentList<ImPlotPlot> footPositionErrors = new SideDependentList<>(new ImPlotPlot(), new ImPlotPlot());
   private final SideDependentList<ImPlotBasicDoublePlotLine> currentFootPositionErrorPlotLines = new SideDependentList<>(new ImPlotBasicDoublePlotLine(),
                                                                                                                          new ImPlotBasicDoublePlotLine());
   private final SideDependentList<ImPlotBasicDoublePlotLine> desiredFootPositionErrorPlotLines = new SideDependentList<>(new ImPlotBasicDoublePlotLine(),
                                                                                                                          new ImPlotBasicDoublePlotLine());
   private final SideDependentList<ImPlotPlot> footOrientationErrors = new SideDependentList<>(new ImPlotPlot(), new ImPlotPlot());
   private final SideDependentList<ImPlotBasicDoublePlotLine> currentFootOrientationErrorPlotLines = new SideDependentList<>(new ImPlotBasicDoublePlotLine(),
                                                                                                                             new ImPlotBasicDoublePlotLine());
   private final SideDependentList<ImPlotBasicDoublePlotLine> desiredFootOrientationErrorPlotLines = new SideDependentList<>(new ImPlotBasicDoublePlotLine(),
                                                                                                                             new ImPlotBasicDoublePlotLine());
   private final ImPlotPlot handForcePlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine handForcePlotLine = new ImPlotBasicDoublePlotLine();
   private final ImPlotPlot handTorquePlot = new ImPlotPlot();
   private final ImPlotBasicDoublePlotLine handTorquePlotLine = new ImPlotBasicDoublePlotLine();
   private int emptyPlotIndex;
   private double elapsedExecutionTime = -1.0;
   private boolean newlyExecuting = false;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator
         = new MultipleWaypointsPositionTrajectoryGenerator("Position", 500, ReferenceFrame.getWorldFrame(), new YoRegistry("DummyParent"));
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator
         = new MultipleWaypointsOrientationTrajectoryGenerator("Orientation", 500, ReferenceFrame.getWorldFrame(), new YoRegistry("DummyParent"));
   private final MultipleWaypointsTrajectoryGenerator jointspaceTrajectoryGenerator
         = new MultipleWaypointsTrajectoryGenerator("Jointspace", 500, new YoRegistry("DummyParent"));
   private boolean elapsedTimeIsValid;

   public RDXActionProgressWidgets(RDXActionNode<?, ?> action)
   {
      this.action = action;

      setupPlot(positionErrorPlot, 0.1, currentPositionErrorPlotLine, desiredPositionErrorPlotLine);
      setupPlot(orientationErrorPlot, 45.0, currentOrientationPlotLine, desiredOrientationPlotLine);
      for (int i = 0; i < jointspacePositionErrorPlot.length; i++)
      {
         jointspacePositionErrorPlot[i] = new ImPlotPlot();
         currentJointspacePositionErrorPlotLine[i] = new ImPlotBasicDoublePlotLine();
         desiredJointspacePositionErrorPlotLine[i] = new ImPlotBasicDoublePlotLine();
         setupPlot(jointspacePositionErrorPlot[i], 5.0, currentJointspacePositionErrorPlotLine[i], desiredJointspacePositionErrorPlotLine[i]);
      }
      setupPlot(footstepsRemainingPlot, 1.0, footstepsRemainingPlotLine);
      for (RobotSide side : RobotSide.values)
      {
         setupPlot(footPositionErrors.get(side), 0.1, currentFootPositionErrorPlotLines.get(side), desiredFootPositionErrorPlotLines.get(side));
         setupPlot(footOrientationErrors.get(side), 0.1, currentFootOrientationErrorPlotLines.get(side), desiredFootOrientationErrorPlotLines.get(side));
      }
      setupPlot(handForcePlot, 50.0, handForcePlotLine);
      setupPlot(handTorquePlot, 5.0, handTorquePlotLine);
   }

   private void setupPlot(ImPlotPlot plot, double limitYMin, ImPlotBasicDoublePlotLine... plotLines)
   {
      plot.setFlag(ImPlotFlags.NoLegend);
      for (ImPlotBasicDoublePlotLine plotLine : plotLines)
      {
         plot.getPlotLines().add(plotLine);
      }
      plot.setCustomBeforePlotLogic(() ->
      {
         double max = Double.NaN;
         for (ImPlotBasicDoublePlotLine plotLine : plotLines)
         {
            double plotMaxY = plotLine.getMaxYValue();
            max = Double.isNaN(max) ? plotMaxY : Math.max(plotMaxY, max);
         }
         ImPlot.setNextPlotLimitsY(0.0, Double.isNaN(max) ? limitYMin : Double.max(limitYMin, max), ImGuiCond.Always);
      });
   }

   public void update()
   {
      emptyPlotIndex = 0;
      double newElapsedExecutionTime = action.getState().getElapsedExecutionTime();
      newlyExecuting = newElapsedExecutionTime < elapsedExecutionTime;
      elapsedExecutionTime = newElapsedExecutionTime;
      elapsedTimeIsValid = !Double.isNaN(elapsedExecutionTime);

      if (newlyExecuting)
      {
         currentPositionErrorPlotLine.clear();
         desiredPositionErrorPlotLine.clear();
         currentOrientationPlotLine.clear();
         desiredOrientationPlotLine.clear();
         for (int i = 0; i < jointspacePositionErrorPlot.length; i++)
         {
            currentJointspacePositionErrorPlotLine[i].clear();
            desiredJointspacePositionErrorPlotLine[i].clear();
         }
         footstepsRemainingPlotLine.clear();
         for (RobotSide side : RobotSide.values)
         {
            currentFootPositionErrorPlotLines.get(side).clear();
            desiredFootPositionErrorPlotLines.get(side).clear();
            currentFootOrientationErrorPlotLines.get(side).clear();
            desiredFootOrientationErrorPlotLines.get(side).clear();
         }
         handForcePlotLine.clear();
         handTorquePlotLine.clear();
      }
   }

   public void renderElapsedTimeBar(float dividedBarWidth)
   {
      double nominalDuration = action.getState().getNominalExecutionDuration();
      double percentComplete = elapsedExecutionTime / nominalDuration;
      double percentLeft = 1.0 - percentComplete;
      ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, "%.2f / %.2f".formatted(elapsedExecutionTime, nominalDuration));
   }

   public void renderPositionError(float dividedBarWidth, boolean renderAsPlots)
   {
      if (!action.getState().getDesiredTrajectory().isEmpty())
      {
         if (elapsedTimeIsValid)
         {
            positionTrajectoryGenerator.clear();
            for (int i = 0; i < action.getState().getDesiredTrajectory().getSize(); i++)
            {
               positionTrajectoryGenerator.appendWaypoint(action.getState().getDesiredTrajectory().getValueReadOnly(i));
            }
            positionTrajectoryGenerator.initialize();
            positionTrajectoryGenerator.compute(elapsedExecutionTime);
         }

         Point3DReadOnly initialPosition = action.getState().getDesiredTrajectory().getFirstValueReadOnly().getPosition();
         Point3DReadOnly endPosition = action.getState().getDesiredTrajectory().getLastValueReadOnly().getPosition();
         Point3DReadOnly currentPosition = action.getState().getCurrentPose().getValueReadOnly().getPosition();
         Point3DReadOnly desiredPosition = elapsedTimeIsValid ? positionTrajectoryGenerator.getPosition() : endPosition;

         double initialToEnd = initialPosition.differenceNorm(endPosition);
         double currentToEnd = currentPosition.differenceNorm(endPosition);
         double desiredToEnd = desiredPosition.differenceNorm(endPosition);
         double tolerance = action.getState().getPositionDistanceToGoalTolerance();
         double error = Math.abs(currentToEnd - desiredToEnd);
         int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            currentPositionErrorPlotLine.setDataColor(dataColor);
            currentPositionErrorPlotLine.addValue(currentToEnd);
            desiredPositionErrorPlotLine.setDataColor(ImGuiTools.GRAY);
            desiredPositionErrorPlotLine.addValue(desiredToEnd);
         }
         if (renderAsPlots)
         {
            positionErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
            double toleranceMarkPercent = tolerance / barEndValue;
            double percentLeft = currentToEnd / barEndValue;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                         dividedBarWidth,
                                         dataColor,
                                         percentLeft,
                                         toleranceMarkPercent,
                                         "%.2f / %.2f".formatted(currentToEnd, initialToEnd));
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderOrientationError(float dividedBarWidth, boolean renderAsPlots)
   {
      if (!action.getState().getDesiredTrajectory().isEmpty())
      {
         if (elapsedTimeIsValid)
         {
            orientationTrajectoryGenerator.clear();
            for (int i = 0; i < action.getState().getDesiredTrajectory().getSize(); i++)
            {
               orientationTrajectoryGenerator.appendWaypoint(action.getState().getDesiredTrajectory().getValueReadOnly(i));
            }
            orientationTrajectoryGenerator.initialize();
            orientationTrajectoryGenerator.compute(elapsedExecutionTime);
         }

         QuaternionReadOnly initialOrientation = action.getState().getDesiredTrajectory().getFirstValueReadOnly().getOrientation();
         QuaternionReadOnly endOrientation = action.getState().getDesiredTrajectory().getLastValueReadOnly().getOrientation();
         QuaternionReadOnly currentOrientation = action.getState().getCurrentPose().getValueReadOnly().getOrientation();
         QuaternionReadOnly desiredOrientation = elapsedTimeIsValid ? orientationTrajectoryGenerator.getOrientation(): endOrientation;

         double initialToEnd = initialOrientation.distance(endOrientation, true);
         double currentToEnd = currentOrientation.distance(endOrientation, true);
         double desiredToEnd = desiredOrientation.distance(endOrientation, true);
         double tolerance = action.getState().getOrientationDistanceToGoalTolerance();
         double error = Math.abs(currentToEnd - desiredToEnd);
         int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            currentOrientationPlotLine.setDataColor(dataColor);
            currentOrientationPlotLine.addValue(Math.toDegrees(currentToEnd));
            desiredOrientationPlotLine.setDataColor(ImGuiTools.GRAY);
            desiredOrientationPlotLine.addValue(Math.toDegrees(desiredToEnd));
         }
         if (renderAsPlots)
         {
            orientationErrorPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
            double toleranceMarkPercent = tolerance / barEndValue;
            double percentLeft = currentToEnd / barEndValue;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                         dividedBarWidth,
                                         dataColor,
                                         percentLeft,
                                         toleranceMarkPercent,
                                         "%.2f / %.2f".formatted(Math.toDegrees(currentToEnd), Math.toDegrees(initialToEnd)));
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderJointspacePositionError(int jointIndex, float dividedBarWidth, boolean renderAsPlots)
   {
      if (action.getState().getDesiredJointTrajectories().getNumberOfJoints() > jointIndex)
      {
         if (elapsedTimeIsValid)
         {
            jointspaceTrajectoryGenerator.clear();
            for (int i = 0; i < action.getState().getDesiredJointTrajectories().getNumberOfPoints(jointIndex); i++)
            {
               jointspaceTrajectoryGenerator.appendWaypoint(action.getState().getDesiredJointTrajectories().getValueReadOnly(jointIndex, i));
            }
            jointspaceTrajectoryGenerator.initialize();
            jointspaceTrajectoryGenerator.compute(elapsedExecutionTime);
         }

         double initialPosition = action.getState().getDesiredJointTrajectories().getFirstValueReadOnly(jointIndex).getPosition();
         double endPosition = action.getState().getDesiredJointTrajectories().getLastValueReadOnly(jointIndex).getPosition();
         double currentPosition = action.getState().getCurrentJointAngles().getValueReadOnly(jointIndex);
         double desiredPosition = elapsedTimeIsValid ? jointspaceTrajectoryGenerator.getValue() : endPosition;

         double initialToEnd = Math.abs(initialPosition - endPosition);
         double currentToEnd = Math.abs(currentPosition - endPosition);
         double desiredToEnd = Math.abs(desiredPosition - endPosition);
         double tolerance = action.getState().getPositionDistanceToGoalTolerance();
         double error = Math.abs(currentToEnd - desiredToEnd);
         int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            currentJointspacePositionErrorPlotLine[jointIndex].setDataColor(dataColor);
            currentJointspacePositionErrorPlotLine[jointIndex].addValue(Math.toDegrees(currentToEnd));
            desiredJointspacePositionErrorPlotLine[jointIndex].setDataColor(ImGuiTools.GRAY);
            desiredJointspacePositionErrorPlotLine[jointIndex].addValue(Math.toDegrees(desiredToEnd));
         }
         if (renderAsPlots)
         {
            jointspacePositionErrorPlot[jointIndex].render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
            double toleranceMarkPercent = tolerance / barEndValue;
            double percentLeft = currentToEnd / barEndValue;
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                         dividedBarWidth,
                                         dataColor,
                                         percentLeft,
                                         toleranceMarkPercent,
                                         "%.2f / %.2f".formatted(Math.toDegrees(currentToEnd), Math.toDegrees(initialToEnd)));
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderHandForce(float dividedBarWidth, boolean renderAsPlots)
   {
      CRDTUnidirectionalVector3D forceCRDT = null;
      if (action instanceof RDXHandPoseAction handPoseAction)
         forceCRDT = handPoseAction.getState().getForce();
      else if (action instanceof RDXScrewPrimitiveAction screwPrimitiveAction)
         forceCRDT = screwPrimitiveAction.getState().getForce();

      if (forceCRDT != null)
      {
         double limit = 20.0;
         double force = forceCRDT.getValueReadOnly().norm();
         int dataColor = force < limit ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            handForcePlotLine.setDataColor(dataColor);
            handForcePlotLine.addValue(force);
         }
         if (renderAsPlots)
         {
            handForcePlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT, dividedBarWidth, dataColor, force / limit, 0.5, "%.2f".formatted(force));
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderHandTorque(float dividedBarWidth, boolean renderAsPlots)
   {
      CRDTUnidirectionalVector3D torqueCRDT = null;
      if (action instanceof RDXHandPoseAction handPoseAction)
         torqueCRDT = handPoseAction.getState().getTorque();
      else if (action instanceof RDXScrewPrimitiveAction screwPrimitiveAction)
         torqueCRDT = screwPrimitiveAction.getState().getTorque();

      if (torqueCRDT != null)
      {
         double limit = 20.0;
         double torque = torqueCRDT.getValueReadOnly().norm();
         int dataColor = torque < limit ? ImGuiTools.GREEN : ImGuiTools.RED;

         if (action.getState().getIsExecuting())
         {
            handTorquePlotLine.setDataColor(dataColor);
            handTorquePlotLine.addValue(torque);
         }
         if (renderAsPlots)
         {
            handTorquePlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT, dividedBarWidth, dataColor, torque / limit, 0.5, "%.2f".formatted(torque));
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderFootstepCompletion(float dividedBarWidth, boolean renderAsPlots)
   {
      if (action instanceof RDXFootstepPlanAction footstepPlanAction && footstepPlanAction.getState().getTotalNumberOfFootsteps() > 0)
      {
         FootstepPlanActionState footstepPlanActionState = footstepPlanAction.getState();
         double percentLeft = footstepPlanActionState.getNumberOfIncompleteFootsteps() / (double) footstepPlanActionState.getTotalNumberOfFootsteps();
         String overlay = "%d / %d".formatted(footstepPlanActionState.getNumberOfIncompleteFootsteps(), footstepPlanActionState.getTotalNumberOfFootsteps());

         if (action.getState().getIsExecuting())
         {
            footstepsRemainingPlotLine.addValue(footstepPlanActionState.getNumberOfIncompleteFootsteps());
         }
         if (renderAsPlots)
         {
            footstepsRemainingPlot.render(dividedBarWidth, PLOT_HEIGHT);
         }
         else
         {
            ImGui.progressBar((float) percentLeft, dividedBarWidth, PROGRESS_BAR_HEIGHT, overlay);
         }
      }
      else
      {
         renderBlankProgress(dividedBarWidth, renderAsPlots, true);
      }
   }

   public void renderFootPositions(float dividedBarWidth, boolean renderAsPlots)
   {
      float halfDividedBarWidth = dividedBarWidth / 2.0f - ImGui.getStyle().getItemSpacingX() / 2.0f;

      for (RobotSide side : RobotSide.values)
      {
         if (action instanceof RDXFootstepPlanAction footstepPlanAction
             && !footstepPlanAction.getState().getDesiredFootPoses().get(side).isEmpty())
         {
            FootstepPlanActionState footstepPlanActionState = footstepPlanAction.getState();
            int i = 0;
            SE3TrajectoryPointReadOnly nextDesiredPoint = footstepPlanActionState.getDesiredFootPoses().get(side).getValueReadOnly(i++);
            while (i < footstepPlanActionState.getDesiredFootPoses().get(side).getSize()
                && nextDesiredPoint.getTime() < elapsedExecutionTime)
               nextDesiredPoint = footstepPlanActionState.getDesiredFootPoses().get(side).getValueReadOnly(i++);

            Point3DReadOnly initialPosition = footstepPlanActionState.getDesiredFootPoses().get(side).getFirstValueReadOnly().getPosition();
            Point3DReadOnly endPosition = footstepPlanActionState.getDesiredFootPoses().get(side).getLastValueReadOnly().getPosition();
            Point3DReadOnly currentPosition = footstepPlanActionState.getCurrentFootPoses().get(side).getValueReadOnly().getPosition();
            Point3DReadOnly desiredPosition = nextDesiredPoint.getPosition();

            double initialToEnd = initialPosition.differenceNorm(endPosition);
            double currentToEnd = currentPosition.differenceNorm(endPosition);
            double desiredToEnd = desiredPosition.differenceNorm(endPosition);
            double tolerance = action.getState().getPositionDistanceToGoalTolerance();
            double error = Math.abs(currentToEnd - desiredToEnd);
            int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

            if (action.getState().getIsExecuting())
            {
               currentFootPositionErrorPlotLines.get(side).setDataColor(dataColor);
               currentFootPositionErrorPlotLines.get(side).addValue(currentToEnd);
               desiredFootPositionErrorPlotLines.get(side).setDataColor(ImGuiTools.GRAY);
               desiredFootPositionErrorPlotLines.get(side).addValue(desiredToEnd);
            }
            if (renderAsPlots)
            {
               footPositionErrors.get(side).render(halfDividedBarWidth, PLOT_HEIGHT);
            }
            else
            {
               double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
               double toleranceMarkPercent = tolerance / barEndValue;
               double percentLeft = currentToEnd / barEndValue;
               ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                            halfDividedBarWidth,
                                            dataColor,
                                            percentLeft,
                                            toleranceMarkPercent,
                                            "%.2f / %.2f".formatted(currentToEnd, initialToEnd));
            }
         }
         else
         {
            renderBlankProgress(dividedBarWidth, renderAsPlots, true);
         }

         if (side == RobotSide.LEFT)
            ImGui.sameLine();
      }
   }

   public void renderFootOrientations(float dividedBarWidth, boolean renderAsPlots)
   {
      float halfDividedBarWidth = dividedBarWidth / 2.0f - ImGui.getStyle().getItemSpacingX() / 2.0f;

      for (RobotSide side : RobotSide.values)
      {
         if (action instanceof RDXFootstepPlanAction footstepPlanAction
             && !footstepPlanAction.getState().getDesiredFootPoses().get(side).isEmpty())
         {
            FootstepPlanActionState footstepPlanActionState = footstepPlanAction.getState();
            int i = 0;
            SE3TrajectoryPointReadOnly nextDesiredPoint = footstepPlanActionState.getDesiredFootPoses().get(side).getValueReadOnly(i++);
            while (i < footstepPlanActionState.getDesiredFootPoses().get(side).getSize()
                   && nextDesiredPoint.getTime() < elapsedExecutionTime)
               nextDesiredPoint = footstepPlanActionState.getDesiredFootPoses().get(side).getValueReadOnly(i++);

            QuaternionReadOnly initialOrientation = footstepPlanActionState.getDesiredFootPoses().get(side).getFirstValueReadOnly().getOrientation();
            QuaternionReadOnly endOrientation = footstepPlanActionState.getDesiredFootPoses().get(side).getLastValueReadOnly().getOrientation();
            QuaternionReadOnly currentOrientation = footstepPlanActionState.getCurrentFootPoses().get(side).getValueReadOnly().getOrientation();
            QuaternionReadOnly desiredOrientation = nextDesiredPoint.getOrientation();

            double initialToEnd = initialOrientation.distance(endOrientation, true);
            double currentToEnd = currentOrientation.distance(endOrientation, true);
            double desiredToEnd = desiredOrientation.distance(endOrientation, true);
            double tolerance = action.getState().getOrientationDistanceToGoalTolerance();
            double error = Math.abs(currentToEnd - desiredToEnd);
            int dataColor = error < tolerance ? ImGuiTools.GREEN : ImGuiTools.RED;

            if (action.getState().getIsExecuting())
            {
               currentFootOrientationErrorPlotLines.get(side).setDataColor(dataColor);
               currentFootOrientationErrorPlotLines.get(side).addValue(currentToEnd);
               desiredFootOrientationErrorPlotLines.get(side).setDataColor(ImGuiTools.GRAY);
               desiredFootOrientationErrorPlotLines.get(side).addValue(desiredToEnd);
            }
            if (renderAsPlots)
            {
               footOrientationErrors.get(side).render(halfDividedBarWidth, PLOT_HEIGHT);
            }
            else
            {
               double barEndValue = Math.max(Math.min(initialToEnd, currentToEnd), 2.0 * tolerance);
               double toleranceMarkPercent = tolerance / barEndValue;
               double percentLeft = currentToEnd / barEndValue;
               ImGuiTools.markedProgressBar(PROGRESS_BAR_HEIGHT,
                                            halfDividedBarWidth,
                                            dataColor,
                                            percentLeft,
                                            toleranceMarkPercent,
                                            "%.2f / %.2f".formatted(currentToEnd, initialToEnd));
            }
         }
         else
         {
            renderBlankProgress(dividedBarWidth, renderAsPlots, true);
         }

         if (side == RobotSide.LEFT)
            ImGui.sameLine();
      }
   }

   public void renderBlankProgress(float width, boolean renderAsPlots, boolean supportsPlots)
   {
      renderBlankProgress(labels.get("Empty Plot", emptyPlotIndex++), width, renderAsPlots, supportsPlots);
   }

   public static void renderBlankProgress(String emptyPlotLabel, float width, boolean renderAsPlots, boolean supportsPlots)
   {
      if (renderAsPlots && supportsPlots)
      {
         ImPlotTools.renderEmptyPlotArea(emptyPlotLabel, width, RDXActionProgressWidgets.PLOT_HEIGHT);
      }
      else
      {
         ImGui.progressBar(Float.NaN, width, RDXActionProgressWidgets.PROGRESS_BAR_HEIGHT, "");
      }
   }
}