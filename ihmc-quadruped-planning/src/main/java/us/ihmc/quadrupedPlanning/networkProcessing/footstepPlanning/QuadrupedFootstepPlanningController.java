package us.ihmc.quadrupedPlanning.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.footstepPlanning.QuadrupedBodyPathAndFootstepPlanner;
import us.ihmc.quadrupedPlanning.footstepPlanning.turnWalkTurn.QuadrupedSplineWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedPlanning.networkProcessing.OutputManager;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedFootstepPlanningController extends QuadrupedToolboxController
{
   private final QuadrupedBodyPathAndFootstepPlanner planner;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final AtomicLong robotTimestampNanos = new AtomicLong();
   private final YoDouble robotTimestamp = new YoDouble("robotTimestamp", registry);

   public QuadrupedFootstepPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                              OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver, YoVariableRegistry parentRegistry,
                                              YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);


      planner = new QuadrupedSplineWithTurnWalkTurnPlanner(xGaitSettings, robotTimestamp, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(),
                                                           graphicsListRegistry, registry);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      planner.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      planner.setGroundPlane(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processRobotTimestamp(long timestampInNanos)
   {
      this.robotTimestampNanos.set(timestampInNanos);
   }

   public void processFootstepPlanningRequest(QuadrupedFootstepPlanningRequestPacket footstepPlanningRequestPacket)
   {
      FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlanningRequestPacket.getBodyPositionInWorld(),
                                                footstepPlanningRequestPacket.getBodyOrientationInWorld());
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlanningRequestPacket.getGoalPositionInWorld(),
                                             footstepPlanningRequestPacket.getGoalOrientationInWorld());

      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      goal.setGoalPose(goalPose);

      processPlanarRegionsListMessage(footstepPlanningRequestPacket.getPlanarRegionsListMessage());

      planner.setInitialBodyPose(initialPose);
      planner.setGoal(goal);

      planner.planPath();
      planner.plan();
   }


   @Override
   public boolean initializeInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      planner.initialize();

      return true;
   }

   @Override
   public void updateInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      planner.update();

      reportMessage(convertToMessage(planner.getSteps()));
   }

   @Override
   public boolean isDone()
   {
      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }

   private static QuadrupedTimedStepListMessage convertToMessage( List<? extends QuadrupedTimedStep> steps)
   {
      if (steps == null)
         return null;

      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
      {
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));
      }

      return QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
   }
}
