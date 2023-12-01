package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.DesiredExternalWrenchHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int dofs = Twist.SIZE;
   private static final FrameVector3D zeroInWorld = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   private ControllerCoreOutputReadOnly controllerCoreOutput;
   private final SelectionMatrix6D accelerationSelectionMatrix = new SelectionMatrix6D();

   private final FramePose3D bodyFixedControlledPose = new FramePose3D();
   private final SpatialAcceleration bodyAcceleration;

   private final FramePoint3D previousContactPoint = new FramePoint3D();
   private final FrameVector3D previousContactNormal = new FrameVector3D();
   private double previousCoefficientOfFriction;

   private final YoFramePoint3D contactPoint;
   private final YoFramePoint3D contactPointInWorld;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;

   private final ContactablePlaneBody contactableBody;
   private final YoDouble coefficientOfFriction;
   private final YoFrameVector3D contactNormal;
   private final ReferenceFrame contactFrame;
   private final YoEnum<LoadBearingControlMode> defaultControlMode;
   private final YoEnum<LoadBearingControlMode> controlMode;
   private final Vector3DReadOnly handLoadedAccelerationWeight;

   private final RigidBodyTransform bodyToJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform contactToJointTransform = new RigidBodyTransform();

   private final FramePoint3D desiredContactPosition = new FramePoint3D(worldFrame);
   private final FrameQuaternion desiredContactOrientation = new FrameQuaternion(worldFrame);

   private final RigidBodyJointControlHelper jointControlHelper;
   private final RigidBodyOrientationControlHelper orientationControlHelper;

   public RigidBodyLoadBearingControlState(RigidBodyBasics bodyToControl,
                                           ContactablePlaneBody contactableBody,
                                           RigidBodyBasics elevator,
                                           YoDouble yoTime,
                                           RigidBodyJointControlHelper jointControlHelper,
                                           RigidBodyOrientationControlHelper orientationControlHelper,
                                           Vector3DReadOnly handLoadedAccelerationWeight,
                                           YoGraphicsListRegistry graphicsListRegistry,
                                           YoRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.contactFrame = contactableBody.getSoleFrame();
      this.contactableBody = contactableBody;

      bodyFrame.getTransformToDesiredFrame(bodyToJointTransform, bodyToControl.getParentJoint().getFrameAfterJoint());
      bodyAcceleration = new SpatialAcceleration(contactFrame, elevatorFrame, contactFrame);

      spatialAccelerationCommand.set(elevator, bodyToControl);
      this.handLoadedAccelerationWeight = handLoadedAccelerationWeight;
      accelerationSelectionMatrix.getAngularPart().clearSelection();

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new YoDouble(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector3D(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint3D(bodyName + "ContactPoint", contactFrame, parentRegistry);
      contactPointInWorld = new YoFramePoint3D(bodyName + "ContactPointInWorld", worldFrame, parentRegistry);

      defaultControlMode = new YoEnum<>(bodyName + "DefaultControlMode", parentRegistry, LoadBearingControlMode.class);
      controlMode = new YoEnum<>(bodyName + "ControlMode", parentRegistry, LoadBearingControlMode.class);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);

      this.jointControlHelper = jointControlHelper;
      this.orientationControlHelper = orientationControlHelper;

      setupViz(graphicsListRegistry, bodyName);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();

      YoGraphicVector surfaceNormal = new YoGraphicVector(bodyName + "ContactNormal", contactPointInWorld, contactNormal, 0.1, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, surfaceNormal);
      graphics.add(surfaceNormal);

      YoGraphicPosition contactPoint = new YoGraphicPosition(bodyName + "ContactPoint", contactPointInWorld, 0.01, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, contactPoint);
      graphics.add(contactPoint);

      hideGraphics();
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalInWorldFrame(Vector3D contactNormalInWorldFrame)
   {
      contactNormal.set(contactNormalInWorldFrame);
   }

   public void setAndUpdateContactFrame(RigidBodyTransform bodyToContactFrame)
   {
      contactToJointTransform.set(bodyToJointTransform);
      contactToJointTransform.multiply(bodyToContactFrame);
      contactableBody.setSoleFrameTransformFromParentJoint(contactToJointTransform);
      contactPoint.setToZero();
   }

   @Override
   public void doAction(double timeInState)
   {
      // assemble contact command
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.addPointInContact(contactPoint);
      planeContactStateCommand.setHasContactStateChanged(hasContactStateNotChanged());

      // assemble zero acceleration command
      bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
      bodyAcceleration.setBodyFrame(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(contactFrame, bodyAcceleration);
      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);

      // assemble feedback control command
      bodyFixedControlledPose.setToZero(contactFrame);
      bodyFixedControlledPose.changeFrame(bodyFrame);

      double timeInTrajectory = getTimeInTrajectory();

      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
      {
         jointControlHelper.doAction(timeInTrajectory);
      }
      else
      {
         orientationControlHelper.doAction(timeInTrajectory);
         spatialAccelerationCommand.getWeightMatrix().getLinearPart().setWeights(handLoadedAccelerationWeight);
      }

      previousContactNormal.setIncludingFrame(contactNormal);
      previousContactPoint.setIncludingFrame(contactPoint);
      previousCoefficientOfFriction = coefficientOfFriction.getDoubleValue();

      updateGraphics();
   }

   public boolean handleLoadbearingCommand(LoadBearingCommand command)
   {
      setCoefficientOfFriction(command.getCoefficientOfFriction());
      setContactNormalInWorldFrame(command.getContactNormalInWorldFrame());
      setAndUpdateContactFrame(command.getBodyFrameToContactFrame());
      return true;
   }

   public boolean handleJointTrajectoryCommand(JointspaceTrajectoryCommand jointspaceCommand, double[] initialJointPositions)
   {
      if (jointControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use jointspace mode. Was not created with a jointspace helper.");
         return false;
      }

      if (!handleCommandInternal(jointspaceCommand))
      {
         return false;
      }

      if (!jointControlHelper.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         return false;
      }

      controlMode.set(LoadBearingControlMode.JOINTSPACE);
      return true;
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand orientationCommand)
   {
      if (orientationControlHelper == null)
      {
         LogTools.warn(warningPrefix + "Cannot use orientation mode. Was not created with an orientation helper.");
         return false;
      }

      if (!handleCommandInternal(orientationCommand))
      {
         return false;
      }

      if (!orientationControlHelper.handleTrajectoryCommand(orientationCommand))
      {
         return false;
      }

      controlMode.set(LoadBearingControlMode.ORIENTATION);
      return true;
   }

   @Override
   public void onEntry()
   {
      desiredContactPosition.setToZero(contactFrame);
      desiredContactOrientation.setToZero(contactFrame);

      desiredContactPosition.changeFrame(worldFrame);
      desiredContactOrientation.changeFrame(worldFrame);

      previousContactPoint.setToZero(worldFrame);
      previousContactNormal.setToZero(worldFrame);
      previousCoefficientOfFriction = 0.0;
   }

   private boolean hasContactStateNotChanged()
   {
      boolean hasContactStateNotChanged = previousContactNormal.equals(contactNormal);
      hasContactStateNotChanged &= previousContactPoint.equals(contactPoint);
      hasContactStateNotChanged &= previousCoefficientOfFriction == coefficientOfFriction.getDoubleValue();

      return hasContactStateNotChanged;
   }

   @Override
   public void onExit(double timeInState)
   {
      hideGraphics();
      controlMode.set(defaultControlMode.getValue());
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
//      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      else
         feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
      return feedbackControlCommandList;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      if (controlMode.getValue() == LoadBearingControlMode.JOINTSPACE)
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
      else
         feedbackControlCommandList.addCommand(orientationControlHelper.getFeedbackControlCommand());
      return feedbackControlCommandList;
   }

   @Override
   public InverseDynamicsCommand<?> getTransitionOutOfStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setHasContactStateChanged(true);
      return planeContactStateCommand;
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(contactPoint.getNamePrefix(), contactPointInWorld, 0.01, ColorDefinitions.Black()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(contactNormal.getNamePrefix(),
                                                                    contactPointInWorld,
                                                                    contactNormal,
                                                                    0.1,
                                                                    ColorDefinitions.Black()));
      return group;
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      this.controllerCoreOutput = controllerCoreOutput;
   }

   public FramePoint3DReadOnly getContactPoint()
   {
      return contactPoint;
   }

   public FrameVector3DReadOnly getContactNormal()
   {
      return contactNormal;
   }

   public LoadBearingControlMode getLoadBearingControlMode()
   {
      return controlMode.getValue();
   }

   public void updateWholeBodyContactState(WholeBodyContactState wholeBodyContactStateToUpdate)
   {
      wholeBodyContactStateToUpdate.addContactPoints(planeContactStateCommand);
   }

   public double getJointDesiredPosition(int jointIdx)
   {
      return jointControlHelper.getJointDesiredPosition(jointIdx);
   }
}
