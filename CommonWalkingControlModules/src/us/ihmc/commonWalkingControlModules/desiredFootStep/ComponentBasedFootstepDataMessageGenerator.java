package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerCommandInputManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.status.MessageStatusListener;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingUpdater;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class ComponentBasedFootstepDataMessageGenerator implements MessageStatusListener
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<RobotSide> nextSwingLeg = EnumYoVariable.create("nextSwingLeg", RobotSide.class, registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);

   private final ComponentBasedDesiredFootstepCalculator componentBasedDesiredFootstepCalculator;
   private final ControllerCommandInputManager commandInputManager;

   public ComponentBasedFootstepDataMessageGenerator(ControllerCommandInputManager commandInputManager, WalkingControllerParameters walkingControllerParameters,
         CommonHumanoidReferenceFrames referenceFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT,
         ArrayList<Updatable> updatables, boolean useHeadingAndVelocityScript, YoVariableRegistry parentRegistry)
   {
      this.commandInputManager = commandInputManager;
      componentBasedDesiredFootstepCalculator = createComponentBasedDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, bipedFeet,
            controlDT, updatables, useHeadingAndVelocityScript);

      walk.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (walk.getBooleanValue())
               componentBasedDesiredFootstepCalculator.initialize();
         }
      });

      parentRegistry.addChild(registry);
   }

   @Override
   public void receivedNewMessageStatus(Status status, Class<? extends IHMCRosApiMessage<?>> messageClass, long messageId)
   {
      if (messageClass == FootstepDataMessage.class || messageClass == FootstepDataListMessage.class)
      {
         switch (status)
         {
         case ABORTED:
            walk.set(false);
            break;
         case COMPLETED:
            RobotSide supportLeg = nextSwingLeg.getEnumValue().getOppositeSide();
            componentBasedDesiredFootstepCalculator.initializeDesiredFootstep(supportLeg);

            ModifiableFootstepDataListMessage footsteps = computeNextFootsteps(supportLeg);
            commandInputManager.submitFootstepDataListMessage(footsteps);

            nextSwingLeg.set(supportLeg);
         default:
            break;
         }
      }
   }

   private ModifiableFootstepDataListMessage computeNextFootsteps(RobotSide supportLeg)
   {
      ModifiableFootstepDataListMessage footsteps = new ModifiableFootstepDataListMessage();
      ModifiableFootstepDataMessage footstep = componentBasedDesiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      ModifiableFootstepDataMessage nextFootstep = componentBasedDesiredFootstepCalculator.predictFootstepAfterDesiredFootstep(supportLeg.getOppositeSide(), footstep);
      ModifiableFootstepDataMessage nextNextFootstep = componentBasedDesiredFootstepCalculator.predictFootstepAfterDesiredFootstep(supportLeg.getOppositeSide(), nextFootstep);

      footsteps.addFootstep(footstep);
      footsteps.addFootstep(nextFootstep);
      footsteps.addFootstep(nextNextFootstep);
      footsteps.setSwingTime(Double.NaN);
      footsteps.setTransferTime(Double.NaN);

      return footsteps;
   }

   public ComponentBasedDesiredFootstepCalculator createComponentBasedDesiredFootstepCalculator(WalkingControllerParameters walkingControllerParameters,
         CommonHumanoidReferenceFrames referenceFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT,
         ArrayList<Updatable> updatables, boolean useHeadingAndVelocityScript)
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
}
