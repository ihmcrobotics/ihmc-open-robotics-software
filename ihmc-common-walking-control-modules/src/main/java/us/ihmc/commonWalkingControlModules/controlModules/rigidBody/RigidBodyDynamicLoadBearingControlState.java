package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingPostContactState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingPreContactState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingState;
import us.ihmc.commonWalkingControlModules.controlModules.dynamicLoadBearing.DynamicLoadBearingStateEnum;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RigidBodyDynamicLoadBearingControlState extends RigidBodyControlState
{
   private final StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> stateMachine;
   private final DynamicLoadBearingPreContactState preContactState;
   private final DynamicLoadBearingPostContactState postContactState;
   private final RigidBodyJointControlHelper jointControlHelper;
   private final Runnable onExitRunnable;

   public RigidBodyDynamicLoadBearingControlState(RigidBodyBasics bodyToControl,
                                                  RigidBodyBasics baseBody,
                                                  RigidBodyBasics elevator,
                                                  LoadBearingParameters loadBearingParameters,
                                                  YoDouble yoTime,
                                                  Runnable onExitRunnable,
                                                  RigidBodyJointControlHelper jointControlHelper,
                                                  RigidBodyPositionControlHelper positionControlHelper,
                                                  RigidBodyOrientationControlHelper orientationControlHelper,
                                                  ReferenceFrame controlFrame,
                                                  MutableBoolean hasContactChanged,
                                                  YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.DYNAMIC_LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Bracing";

      preContactState = new DynamicLoadBearingPreContactState(bodyToControl, positionControlHelper, controlFrame, registry);
      postContactState = new DynamicLoadBearingPostContactState(loadBearingParameters, bodyToControl, baseBody, elevator, controlFrame, hasContactChanged, registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);

      this.onExitRunnable = onExitRunnable;
      this.jointControlHelper = jointControlHelper;
   }

   private StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<DynamicLoadBearingStateEnum, DynamicLoadBearingState> factory = new StateMachineFactory<>(DynamicLoadBearingStateEnum.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(DynamicLoadBearingStateEnum.PRE_CONTACT, preContactState);
      factory.addState(DynamicLoadBearingStateEnum.POST_CONTACT, postContactState);
      factory.addDoneTransition(DynamicLoadBearingStateEnum.PRE_CONTACT, DynamicLoadBearingStateEnum.POST_CONTACT);

      return factory.build(DynamicLoadBearingStateEnum.PRE_CONTACT);
   }

   public void setBracingSurface(Point3DReadOnly bracingPoint, Vector3DReadOnly bracingNormal, double trajectoryDuration)
   {
      preContactState.setBracingPoint(bracingPoint, bracingNormal, trajectoryDuration);
      postContactState.setBracingSurface(bracingNormal);
   }

   @Override
   public void onEntry()
   {
      stateMachine.resetToInitialState();
   }

   @Override
   public void doAction(double timeInState)
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public void onExit(double timeInState)
   {
      onExitRunnable.run();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return stateMachine.getCurrentStateKey() == DynamicLoadBearingStateEnum.POST_CONTACT && stateMachine.getCurrentState().isDone(stateMachine.getTimeInCurrentState());
   }

   @Override
   public boolean isEmpty()
   {
      // this control mode does not support command queuing
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // this control mode does not support command queuing
      return 0.0;
   }

   public boolean isLoadBearing()
   {
      return stateMachine.getCurrentStateKey() == DynamicLoadBearingStateEnum.POST_CONTACT;
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      if (isLoadBearing())
         postContactState.updateWholeBodyContactState(wholeBodyContactStateToUpdate);
   }

   public void packContactData(RecyclingArrayList<Point3D> contactPointList, Vector3DBasics contactNormalToPack)
   {
      if (isLoadBearing())
      {
         postContactState.packContactData(contactPointList, contactNormalToPack);
      }
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      postContactState.setControllerCoreOutput(controllerCoreOutput);
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      return stateMachine.getCurrentState().getTransitionOutOfStateCommand();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(preContactState.getSCS2YoGraphics());
      return group;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
      for (DynamicLoadBearingStateEnum mode : DynamicLoadBearingStateEnum.values())
      {
         DynamicLoadBearingState state = stateMachine.getState(mode);
         if (state != null && state.createFeedbackControlTemplate() != null)
            feedbackControlCommandList.addCommand(state.createFeedbackControlTemplate());
      }
      return feedbackControlCommandList;
   }
}
