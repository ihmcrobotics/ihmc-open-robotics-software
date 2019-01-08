package us.ihmc.quadrupedPlanning.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.networkProcessing.OutputManager;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedFootstepPlanningController extends QuadrupedToolboxController
{
   private final QuadrupedBodyPathPlanner pathPlanner;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   public QuadrupedFootstepPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                              OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver, YoVariableRegistry parentRegistry,
                                              YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      pathPlanner = new QuadrupedBodyPathPlanner(defaultXGaitSettings, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(),
                                                 graphicsListRegistry, registry);
   }

//   public void setPaused(boolean pause)
//   {
//      pathPlanner.setPaused(pause);
//   }


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
      pathPlanner.processPlanarRegionsListMessage(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      pathPlanner.processGroundPlaneMessage(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      pathPlanner.processXGaitSettingsPacket(packet);
   }

   public void processFootstepPlanningRequest(QuadrupedFootstepPlanningRequestPacket footstepPlanningRequestPacket)
   {
      FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlanningRequestPacket.getBodyPositionInWorld(),
                                                footstepPlanningRequestPacket.getBodyOrientationInWorld());
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPlanningRequestPacket.getGoalPositionInWorld(),
                                             footstepPlanningRequestPacket.getGoalOrientationInWorld());

      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      goal.setGoalPose(goalPose);

      pathPlanner.setInitialBodyPose(initialPose);
      pathPlanner.setGoal(goal);
      pathPlanner.processPlanarRegionsListMessage(footstepPlanningRequestPacket.getPlanarRegionsListMessage());

      pathPlanner.compute();
   }

   public void processTimestamp(long timestampInNanos)
   {
      pathPlanner.processTimestamp(timestampInNanos);
   }


   @Override
   public boolean initializeInternal()
   {
      pathPlanner.initialize();

      return true;
   }

   @Override
   public void updateInternal()
   {
      pathPlanner.update();

      reportMessage(convertToMessage(pathPlanner.getSteps()));
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
