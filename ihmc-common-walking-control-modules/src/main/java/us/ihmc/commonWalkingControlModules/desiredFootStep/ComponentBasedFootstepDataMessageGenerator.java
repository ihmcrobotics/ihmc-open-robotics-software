package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingUpdater;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ComponentBasedFootstepDataMessageGenerator implements Updatable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<RobotSide> nextSwingLeg = YoEnum.create("nextSwingLeg", RobotSide.class, registry);
   private final YoBoolean walk = new YoBoolean("walk", registry);
   private final YoBoolean walkPrevious = new YoBoolean("walkPrevious", registry);

   private final YoDouble swingTime = new YoDouble("footstepGeneratorSwingTime", registry);
   private final YoDouble transferTime = new YoDouble("footstepGeneratorTransferTime", registry);

   private final ComponentBasedDesiredFootstepCalculator componentBasedDesiredFootstepCalculator;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final List<Updatable> updatables = new ArrayList<>();

   public ComponentBasedFootstepDataMessageGenerator(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
         WalkingControllerParameters walkingControllerParameters, HeadingAndVelocityEvaluationScriptParameters scriptParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT, boolean useHeadingAndVelocityScript, HeightMap heightMapForFootZ, YoVariableRegistry parentRegistry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      componentBasedDesiredFootstepCalculator = createComponentBasedDesiredFootstepCalculator(walkingControllerParameters, scriptParameters, referenceFrames, bipedFeet,
            controlDT, useHeadingAndVelocityScript);

      if (heightMapForFootZ != null)
      {
         componentBasedDesiredFootstepCalculator.setGroundProfile(heightMapForFootZ);
      }

      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());

      createFootstepStatusListener();

      parentRegistry.addChild(registry);
   }

   public void computeAndSubmitFootsteps()
   {
      if (!walk.getBooleanValue())
         return;
      RobotSide supportLeg = nextSwingLeg.getEnumValue().getOppositeSide();

      FootstepDataListMessage footsteps = computeNextFootsteps(supportLeg);
      footsteps.setDefaultSwingDuration(swingTime.getDoubleValue());
      footsteps.setDefaultTransferDuration(transferTime.getDoubleValue());
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
      double stepTime = swingTime.getDoubleValue() + transferTime.getDoubleValue();

      componentBasedDesiredFootstepCalculator.initializeDesiredFootstep(supportLeg, stepTime);
      FootstepDataMessage footstep = componentBasedDesiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      FootstepDataMessage nextFootstep = componentBasedDesiredFootstepCalculator
            .predictFootstepAfterDesiredFootstep(supportLeg, footstep, stepTime, stepTime);
      FootstepDataMessage nextNextFootstep = componentBasedDesiredFootstepCalculator
            .predictFootstepAfterDesiredFootstep(supportLeg.getOppositeSide(), nextFootstep, 2.0 * stepTime, stepTime);

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(Double.NaN, Double.NaN);
      footsteps.add(footstep);
      footsteps.add(nextFootstep);
      footsteps.add(nextNextFootstep);
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);

      return footsteps;
   }

   public ComponentBasedDesiredFootstepCalculator createComponentBasedDesiredFootstepCalculator(WalkingControllerParameters walkingControllerParameters, HeadingAndVelocityEvaluationScriptParameters scriptParameters,
         CommonHumanoidReferenceFrames referenceFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT,
         boolean useHeadingAndVelocityScript)
   {
      ManualDesiredVelocityControlModule desiredVelocityControlModule;

      DesiredHeadingControlModule desiredHeadingControlModule;
      if (useHeadingAndVelocityScript)
      {
         desiredVelocityControlModule = new ManualDesiredVelocityControlModule(ReferenceFrame.getWorldFrame(), registry);
         desiredVelocityControlModule.setDesiredVelocity(new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.4, 0.0));

         SimpleDesiredHeadingControlModule simpleDesiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
         simpleDesiredHeadingControlModule.setMaxHeadingDot(0.2);
         simpleDesiredHeadingControlModule.updateDesiredHeadingFrame();
         boolean cycleThroughAllEvents = true;
         HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, controlDT,
               simpleDesiredHeadingControlModule, desiredVelocityControlModule, scriptParameters, registry);
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

      desiredFootstepCalculator.setInPlaceWidth(walkingControllerParameters.getSteppingParameters().getInPlaceWidth());
      desiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getSteppingParameters().getMaxStepLength());
      desiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getSteppingParameters().getMinStepWidth());
      desiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getSteppingParameters().getMaxStepWidth());
      desiredFootstepCalculator.setStepPitch(walkingControllerParameters.getSteppingParameters().getStepPitch());
      return desiredFootstepCalculator;
   }

   @Override
   public void update(double time)
   {
      for(int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }

      if (walk.getBooleanValue() != walkPrevious.getBooleanValue())
      {
         if (walk.getBooleanValue())
         {
            componentBasedDesiredFootstepCalculator.initialize();
            computeAndSubmitFootsteps();
         }
         else
         {
            commandInputManager.submitMessage(HumanoidMessageTools.createPauseWalkingMessage(true));
         }
      }

      walkPrevious.set(walk.getBooleanValue());
   }
}
