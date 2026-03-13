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
   private static final double MINIMUM_TIME_IN_CONTACT = 1.0;
   private static final double CAPTURE_POINT_ERROR_THRESHOLD_TO_REMAIN_IN_STATE = 0.025;

   private final StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> stateMachine;
   private final DynamicLoadBearingPreContactState preContactState;
   private final DynamicLoadBearingPostContactState postContactState;
   private final RigidBodyJointControlHelper jointControlHelper;
   private final Runnable onExitRunnable;
   private final DoubleProvider capturePointErrorProvider;

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
                                                  double nominalRhoWeight,
                                                  DoubleProvider capturePointErrorProvider,
                                                  MutableBoolean hasAddedContacts,
                                                  MutableBoolean hasRemovedContacts,
                                                  YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.DYNAMIC_LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Bracing";

      preContactState = new DynamicLoadBearingPreContactState(bodyToControl, positionControlHelper, controlFrame, registry);
      postContactState = new DynamicLoadBearingPostContactState(bodyToControl,
                                                                baseBody,
                                                                elevator,
                                                                controlFrame,
                                                                positionControlHelper,
                                                                orientationControlHelper,
                                                                loadBearingParameters,
                                                                nominalRhoWeight,
                                                                hasAddedContacts,
                                                                hasRemovedContacts,
                                                                registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);

      this.onExitRunnable = onExitRunnable;
      this.jointControlHelper = jointControlHelper;
      this.capturePointErrorProvider = capturePointErrorProvider;
   }

   private StateMachine<DynamicLoadBearingStateEnum, DynamicLoadBearingState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<DynamicLoadBearingStateEnum, DynamicLoadBearingState> factory = new StateMachineFactory<>(DynamicLoadBearingStateEnum.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(DynamicLoadBearingStateEnum.PRE_CONTACT, preContactState);
      factory.addState(DynamicLoadBearingStateEnum.POST_CONTACT, postContactState);
      factory.addDoneTransition(DynamicLoadBearingStateEnum.PRE_CONTACT, DynamicLoadBearingStateEnum.POST_CONTACT);
//      factory.addDoneTransition(DynamicLoadBearingStateEnum.POST_CONTACT, DynamicLoadBearingStateEnum.PRE_CONTACT);

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
      stateMachine.performTransition(DynamicLoadBearingStateEnum.PRE_CONTACT, false, true);
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
      postContactState.onExit(0.0);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      // Do not exit while in pre-contact phase
      if (stateMachine.getCurrentStateKey() != DynamicLoadBearingStateEnum.POST_CONTACT)
         return false;

      // If the has is slipping or the arm is straightened, exit this state
      boolean isSlippingOrAtSingularity = stateMachine.getCurrentState().isDone(stateMachine.getTimeInCurrentState());
      if (isSlippingOrAtSingularity)
         return true;

      // If the robot has reached a high level of stability, exit this state
      boolean isRecovered = capturePointErrorProvider.getValue() < CAPTURE_POINT_ERROR_THRESHOLD_TO_REMAIN_IN_STATE;
      boolean hasSpentSufficientTimeInContact = stateMachine.getTimeInCurrentState() > MINIMUM_TIME_IN_CONTACT;
      return isRecovered && hasSpentSufficientTimeInContact;
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
      group.addChild(postContactState.getSCS2YoGraphics());
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
