package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingUpdater;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ComponentBasedFootstepDataMessageGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<RobotSide> nextSwingLeg = EnumYoVariable.create("nextSwingLeg", RobotSide.class, registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);

   private final DoubleYoVariable swingTime = new DoubleYoVariable("footstepGeneratorSwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("footstepGeneratorTransferTime", registry);

   private final ComponentBasedDesiredFootstepCalculator componentBasedDesiredFootstepCalculator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final List<Updatable> updatables = new ArrayList<>();

   public ComponentBasedFootstepDataMessageGenerator(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT, boolean useHeadingAndVelocityScript, HeightMap heightMapForFootZ, YoVariableRegistry parentRegistry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      componentBasedDesiredFootstepCalculator = createComponentBasedDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, bipedFeet,
            controlDT, useHeadingAndVelocityScript);

      if (heightMapForFootZ != null)
      {
         componentBasedDesiredFootstepCalculator.setGroundProfile(heightMapForFootZ);
      }

      walk.addVariableChangedListener(createVariableChangedListener());
      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());

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
               componentBasedDesiredFootstepCalculator.initialize();
               computeAndSubmitFootsteps();
            }
            else
            {
               commandInputManager.submitMessage(new PauseWalkingMessage(true));
            }
         }
      };
   }

   public void computeAndSubmitFootsteps()
   {
      if (!walk.getBooleanValue())
         return;
      RobotSide supportLeg = nextSwingLeg.getEnumValue().getOppositeSide();
      componentBasedDesiredFootstepCalculator.initializeDesiredFootstep(supportLeg);

      FootstepDataListCommand footsteps = computeNextFootsteps(supportLeg);
      footsteps.setSwingTime(swingTime.getDoubleValue());
      footsteps.setTransferTime(transferTime.getDoubleValue());
      commandInputManager.submitCommand(footsteps);

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

   private FootstepDataListCommand computeNextFootsteps(RobotSide supportLeg)
   {
      double stepTime = swingTime.getDoubleValue() + transferTime.getDoubleValue();
      FootstepDataListCommand footsteps = new FootstepDataListCommand();
      FootstepDataControllerCommand footstep = componentBasedDesiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      FootstepDataControllerCommand nextFootstep = componentBasedDesiredFootstepCalculator.predictFootstepAfterDesiredFootstep(supportLeg, footstep, stepTime);
      FootstepDataControllerCommand nextNextFootstep = componentBasedDesiredFootstepCalculator.predictFootstepAfterDesiredFootstep(supportLeg.getOppositeSide(), nextFootstep, 2.0 * stepTime);

      footsteps.addFootstep(footstep);
      footsteps.addFootstep(nextFootstep);
      footsteps.addFootstep(nextNextFootstep);
      footsteps.setSwingTime(Double.NaN);
      footsteps.setTransferTime(Double.NaN);

      return footsteps;
   }

   public ComponentBasedDesiredFootstepCalculator createComponentBasedDesiredFootstepCalculator(WalkingControllerParameters walkingControllerParameters,
         CommonHumanoidReferenceFrames referenceFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT,
         boolean useHeadingAndVelocityScript)
   {
      ManualDesiredVelocityControlModule desiredVelocityControlModule;

      DesiredHeadingControlModule desiredHeadingControlModule;
      if (useHeadingAndVelocityScript)
      {
         desiredVelocityControlModule = new ManualDesiredVelocityControlModule(ReferenceFrame.getWorldFrame(), registry);
         desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 1.0, 0.0));

         SimpleDesiredHeadingControlModule simpleDesiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
         simpleDesiredHeadingControlModule.setMaxHeadingDot(0.4);
         simpleDesiredHeadingControlModule.updateDesiredHeadingFrame();
         boolean cycleThroughAllEvents = true;
         HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, controlDT,
               simpleDesiredHeadingControlModule, desiredVelocityControlModule, registry);
         updatables.add(headingAndVelocityEvaluationScript);
         desiredHeadingControlModule = simpleDesiredHeadingControlModule;
      }
      else
      {
         desiredHeadingControlModule = new RateBasedDesiredHeadingControlModule(0.0, controlDT, registry);
         desiredVelocityControlModule = new ManualDesiredVelocityControlModule(desiredHeadingControlModule.getDesiredHeadingFrame(), registry);
      }

      updatables.add(new DesiredHeadingUpdater(desiredHeadingControlModule));

      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = new ComponentBasedDesiredFootstepCalculator(pelvisZUpFrame, bipedFeet,
            desiredHeadingControlModule, desiredVelocityControlModule, registry);

      desiredFootstepCalculator.setInPlaceWidth(walkingControllerParameters.getInPlaceWidth());
      desiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getMaxStepLength());
      desiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getMinStepWidth());
      desiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getMaxStepWidth());
      desiredFootstepCalculator.setStepPitch(walkingControllerParameters.getStepPitch());
      return desiredFootstepCalculator;
   }

   public List<Updatable> getModulesToUpdate()
   {
      return updatables;
   }
}
