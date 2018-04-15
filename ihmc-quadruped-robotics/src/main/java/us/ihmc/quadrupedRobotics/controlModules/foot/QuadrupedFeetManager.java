package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

import java.awt.*;
import java.util.List;

public class QuadrupedFeetManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<QuadrupedFootControlModule> footControlModules = new QuadrantDependentList<>();
   private final QuadrupedControllerToolbox toolbox;

   // support polygon
   private final YoFrameConvexPolygon2D supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoArtifactPolygon supportPolygonVisualizer = new YoArtifactPolygon("supportPolygonVisualizer", supportPolygon, Color.black, false, 1);

   public QuadrupedFeetManager(QuadrupedControllerToolbox toolbox, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footControlModules.set(robotQuadrant, new QuadrupedFootControlModule(robotQuadrant, toolbox, graphicsListRegistry, registry));
      }

      this.toolbox = toolbox;
      graphicsListRegistry.registerArtifact("supportPolygon", supportPolygonVisualizer);
      parentRegistry.addChild(registry);
   }

   public void updateSupportPolygon()
   {
      supportPolygon.clear();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(footControlModules.get(quadrant).getContactState() == ContactState.IN_CONTACT)
            supportPolygon.addVertexMatchingFrame(toolbox.getTaskSpaceEstimates().getSolePosition(quadrant));
      }

      supportPolygon.update();
   }

   public void hideSupportPolygon()
   {
      supportPolygon.clear();
   }

   public YoFrameConvexPolygon2D getSupportPolygon()
   {
      return supportPolygon;
   }

   public void attachStateChangedListener(StateChangedListener<QuadrupedFootStates> stateChangedListener)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         footControlModules.get(quadrant).attachStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(SoleTrajectoryCommand soleTrajectoryCommand, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      initializeWaypointTrajectory(soleTrajectoryCommand.getRobotQuadrant(), soleTrajectoryCommand.getPositionTrajectory().getTrajectoryPointList(),
                                   useInitialSoleForceAsFeedforwardTerm);
   }

   public void initializeWaypointTrajectory(RobotQuadrant robotQuadrant, FrameEuclideanTrajectoryPointList trajectoryPointList, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      QuadrupedFootControlModule footControlModule = footControlModules.get(robotQuadrant);
      if (trajectoryPointList.getNumberOfTrajectoryPoints() > 0)
      {
         footControlModule.requestMoveViaWaypoints();
         footControlModule.initializeWaypointTrajectory(trajectoryPointList, useInitialSoleForceAsFeedforwardTerm);
      }
      else
      {
         footControlModule.requestSupport();
      }
   }

   public void triggerStep(QuadrupedTimedStep step)
   {
      footControlModules.get(step.getRobotQuadrant()).triggerStep(step);
   }

   public void triggerSteps(List<YoQuadrupedTimedStep> steps)
   {
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         footControlModules.get(step.getRobotQuadrant()).triggerStep(step);
      }
   }

   public void adjustSteps(List<QuadrupedStep> activeSteps)
   {
      for (int i = 0; i < activeSteps.size(); i++)
         adjustStep(activeSteps.get(i));
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   public void adjustStep(QuadrupedStep step)
   {
      step.getGoalPosition(tempPoint);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      footControlModules.get(step.getRobotQuadrant()).adjustStep(tempPoint);
   }

   public void adjustStep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly adjustedStep)
   {
      footControlModules.get(robotQuadrant).adjustStep(adjustedStep);
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).reset();
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerStepTransitionCallback(stepTransitionCallback);
   }

   public void registerWaypointCallback(QuadrupedWaypointCallback waypointCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerWaypointCallback(waypointCallback);
   }

   public void compute()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         compute(quadrant);
   }

   public void compute(RobotQuadrant robotQuadrant)
   {
      footControlModules.get(robotQuadrant).compute();
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getContactState();
   }

   public void requestFullContact()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).requestSupport();
   }

   public void requestHoldAll()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).requestHold();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotQuadrant).createFeedbackControlTemplate();
         ret.addCommandList(template);
      }

      return ret;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getFeedbackControlCommand();
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getVirtualModelControlCommand();
   }
}
