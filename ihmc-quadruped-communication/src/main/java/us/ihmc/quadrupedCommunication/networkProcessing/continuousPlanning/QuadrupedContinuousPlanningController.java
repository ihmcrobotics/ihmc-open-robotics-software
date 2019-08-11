package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedContinuousPlanningController extends QuadrupedToolboxController
{
   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedFootstepPlanningToolboxOutputStatus> footstepPlannerOutput = new AtomicReference<>();
   private final AtomicReference<QuadrupedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<QuadrupedFootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   public QuadrupedContinuousPlanningController(OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processFootstepPlannerOutput(QuadrupedFootstepPlanningToolboxOutputStatus message)
   {
      footstepPlannerOutput.set(message);
   }

   public void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket message)
   {
      planningRequestPacket.set(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      footstepStatusMessage.set(message);
   }

   @Override
   public boolean initializeInternal()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {

   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }
}
