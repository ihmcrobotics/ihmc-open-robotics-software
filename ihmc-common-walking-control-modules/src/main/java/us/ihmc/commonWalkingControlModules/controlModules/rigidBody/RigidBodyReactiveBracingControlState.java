package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing.ReactiveBracingPostContactState;
import us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing.ReactiveBracingPreContactState;
import us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing.ReactiveBracingState;
import us.ihmc.commonWalkingControlModules.controlModules.reactiveBracing.ReactiveBracingStateEnum;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RigidBodyReactiveBracingControlState extends RigidBodyControlState
{
   private final StateMachine<ReactiveBracingStateEnum, ReactiveBracingState> stateMachine;
   private final ReactiveBracingPreContactState preContactState;
   private final ReactiveBracingPostContactState postContactState;

   public RigidBodyReactiveBracingControlState(RigidBodyBasics bodyToControl,
                                               RigidBodyBasics baseBody,
                                               RigidBodyBasics elevator,
                                               YoDouble yoTime,
                                               RigidBodyPositionControlHelper positionControlHelper,
                                               RigidBodyOrientationControlHelper orientationControlHelper,
                                               ReferenceFrame controlFrame,
                                               MutableBoolean hasContactChanged,
                                               YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.REACTIVE_BRACING, bodyToControl.getName(), yoTime, parentRegistry);

      String bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Bracing";

      preContactState = new ReactiveBracingPreContactState(bodyToControl, positionControlHelper, controlFrame, registry);
      postContactState = new ReactiveBracingPostContactState(bodyToControl, baseBody, elevator, controlFrame, hasContactChanged, registry);

      stateMachine = setupStateMachine(namePrefix, yoTime);
   }

   private StateMachine<ReactiveBracingStateEnum, ReactiveBracingState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<ReactiveBracingStateEnum, ReactiveBracingState> factory = new StateMachineFactory<>(ReactiveBracingStateEnum.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(ReactiveBracingStateEnum.PRE_CONTACT, preContactState);
      factory.addState(ReactiveBracingStateEnum.POST_CONTACT, postContactState);
      factory.addDoneTransition(ReactiveBracingStateEnum.PRE_CONTACT, ReactiveBracingStateEnum.POST_CONTACT);

      return factory.build(ReactiveBracingStateEnum.PRE_CONTACT);
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

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
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
      return stateMachine.getCurrentStateKey() == ReactiveBracingStateEnum.POST_CONTACT;
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
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
      for (ReactiveBracingStateEnum mode : ReactiveBracingStateEnum.values())
      {
         ReactiveBracingState state = stateMachine.getState(mode);
         if (state != null && state.createFeedbackControlTemplate() != null)
            feedbackControlCommandList.addCommand(state.createFeedbackControlTemplate());
      }
      return feedbackControlCommandList;
   }
}
