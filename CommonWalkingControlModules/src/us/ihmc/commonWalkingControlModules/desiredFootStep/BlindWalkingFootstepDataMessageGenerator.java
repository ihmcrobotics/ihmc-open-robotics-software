package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

// FIXME TODO Get me working please!
public class BlindWalkingFootstepDataMessageGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<RobotSide> nextSwingLeg = EnumYoVariable.create("nextSwingLeg", RobotSide.class, registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);

   private final BlindWalkingDesiredFootstepCalculator blindWalkingDesiredFootstepCalculator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   public BlindWalkingFootstepDataMessageGenerator(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
         WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
         YoVariableRegistry parentRegistry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      blindWalkingDesiredFootstepCalculator = new BlindWalkingDesiredFootstepCalculator(bipedFeet, registry);
      blindWalkingDesiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getMaxStepLength());
      blindWalkingDesiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getMinStepWidth());
      blindWalkingDesiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getMaxStepWidth());
      blindWalkingDesiredFootstepCalculator.setStepPitch(walkingControllerParameters.getStepPitch());

      walk.addVariableChangedListener(createVariableChangedListener());

      createFootstepStatusListener();

      parentRegistry.addChild(registry);
   }

   public VariableChangedListener createVariableChangedListener()
   {
      return new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (walk.getBooleanValue())
            {
               blindWalkingDesiredFootstepCalculator.initialize();
               computeAndSubmitFootsteps();
            }
         }
      };
   }

   public void computeAndSubmitFootsteps()
   {
      RobotSide supportLeg = nextSwingLeg.getEnumValue().getOppositeSide();

      FootstepDataListMessage footsteps = computeNextFootsteps(supportLeg);
      commandInputManager.submitMessage(footsteps);

      nextSwingLeg.set(supportLeg);
   }

   public void createFootstepStatusListener()
   {
      StatusMessageListener<FootstepStatus> footstepStatusListener = new StatusMessageListener<FootstepStatus>()
      {
         @Override
         public void receivedNewMessageStatus(FootstepStatus footstepStatus)
         {
            switch (footstepStatus.status)
            {
            case COMPLETED:
               computeAndSubmitFootsteps();
            default:
               break;
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(FootstepStatus.class, footstepStatusListener);

      StatusMessageListener<WalkingStatusMessage> walkingStatusListener = new StatusMessageListener<WalkingStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(WalkingStatusMessage walkingStatusListener)
         {
            switch (walkingStatusListener.getWalkingStatus())
            {
            case ABORT_REQUESTED:
               walk.set(false);
            default:
               break;
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusListener);
   }

   private FootstepDataListMessage computeNextFootsteps(RobotSide supportLeg)
   {
      double stepTime = 0.0; //TODO get the time right.
      blindWalkingDesiredFootstepCalculator.initializeDesiredFootstep(supportLeg, stepTime);
      FootstepDataMessage footstep = blindWalkingDesiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      FootstepDataMessage nextFootstep = blindWalkingDesiredFootstepCalculator
            .predictFootstepAfterDesiredFootstep(supportLeg, footstep, stepTime, stepTime);
      FootstepDataMessage nextNextFootstep = blindWalkingDesiredFootstepCalculator
            .predictFootstepAfterDesiredFootstep(supportLeg.getOppositeSide(), nextFootstep, 2.0 * stepTime, stepTime);

      FootstepDataListMessage footsteps = new FootstepDataListMessage(Double.NaN, Double.NaN);
      footsteps.add(footstep);
      footsteps.add(nextFootstep);
      footsteps.add(nextNextFootstep);

      return footsteps;
   }
}
