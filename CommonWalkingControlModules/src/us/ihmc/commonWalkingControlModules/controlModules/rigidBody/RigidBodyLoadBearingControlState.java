package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final long NO_CONTACT_ID = 0L;
   private static final long IN_CONTACT_ID = 1L;
   private static final int dofs = Twist.SIZE;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();

   private final DenseMatrix64F accelerationSelectionMatrix = new DenseMatrix64F(dofs, dofs);
   private final DenseMatrix64F feedbackSelectionMatrix = new DenseMatrix64F(dofs, dofs);
   private final boolean[] isDirectionFeedbackControlled = new boolean[dofs];

   private final FramePose bodyFixedControlledPose = new FramePose();
   private final SpatialAccelerationVector bodyAcceleration;

   // TODO: allow multiple surface normals?
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   // TODO: allow multiple contact points
   private final YoFramePoint contactPoint;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final RigidBody body;
   private final ContactablePlaneBody contactableBody;

   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormal;
   private final ReferenceFrame contactFrame;
   private final PoseReferenceFrame desiredContactFrame;

   private final RigidBodyTransform bodyToJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform contactToJointTransform = new RigidBodyTransform();

   private final FramePoint desiredContactPosition = new FramePoint(worldFrame);
   private final FrameOrientation desiredContactOrientation = new FrameOrientation(worldFrame);
   private final FramePoint currentContactPosition = new FramePoint(worldFrame);
   private final FrameOrientation currentContactOrientation = new FrameOrientation(worldFrame);

   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private final ArrayList<YoGraphic> graphics = new ArrayList<>();

   public RigidBodyLoadBearingControlState(RigidBody bodyToControl, ContactablePlaneBody contactableBody, RigidBody elevator, DoubleYoVariable yoTime,
         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.body = bodyToControl;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.contactFrame = contactableBody.getSoleFrame();
      this.contactableBody = contactableBody;

      body.getBodyFixedFrame().getTransformToDesiredFrame(bodyToJointTransform, body.getParentJoint().getFrameAfterJoint());

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);
      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      bodyAcceleration = new SpatialAccelerationVector(contactFrame, elevatorFrame, contactFrame);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
      spatialFeedbackControlCommand.set(elevator, body);

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new DoubleYoVariable(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint(bodyName + "ContactPoint", contactFrame, parentRegistry);
      desiredContactFrame = new PoseReferenceFrame(bodyName + "DesiredContactFrame", worldFrame);

      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(NO_CONTACT_ID);

      setupViz(graphicsListRegistry);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();
      YoGraphicReferenceFrame contactFrameViz = new YoGraphicReferenceFrame(contactFrame, registry, 0.1);
      graphicsListRegistry.registerYoGraphic(listName, contactFrameViz);
      graphics.add(contactFrameViz);

      YoGraphicReferenceFrame desiredContactFrameViz = new YoGraphicReferenceFrame(desiredContactFrame, registry, 0.1);
      graphicsListRegistry.registerYoGraphic(listName, desiredContactFrameViz);
      graphics.add(desiredContactFrameViz);
   }

   public void setWeights(Vector3D taskspaceAngularWeight, Vector3D taskspaceLinearWeight)
   {
      spatialFeedbackControlCommand.setWeightsForSolver(taskspaceAngularWeight, taskspaceLinearWeight);
   }

   public void setGains(YoOrientationPIDGainsInterface taskspaceOrientationGains, YoPositionPIDGainsInterface taskspacePositionGains)
   {
      spatialFeedbackControlCommand.setGains(taskspaceOrientationGains);
      spatialFeedbackControlCommand.setGains(taskspacePositionGains);
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalInWorldFrame(Vector3D contactNormalInWorldFrame)
   {
      contactNormal.set(contactNormalInWorldFrame);
   }

   public void setContactFrame(RigidBodyTransform bodyToContactFrame)
   {
      contactToJointTransform.set(bodyToJointTransform);
      contactToJointTransform.multiply(bodyToContactFrame);
      contactableBody.setSoleFrameTransformFromParentJoint(contactToJointTransform);
      contactPoint.setToZero();
   }

   @Override
   public void doAction()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal.getFrameTuple());
      planeContactStateCommand.addPointInContact(contactPoint.getFrameTuple());
      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(IN_CONTACT_ID);

      bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
      bodyAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFrame);
      bodyAcceleration.changeFrameNoRelativeMotion(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(bodyAcceleration);

      updateDesiredContactPose();
      bodyFixedControlledPose.setToZero(contactFrame);
      bodyFixedControlledPose.changeFrame(bodyFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.set(desiredContactPosition, desiredLinearVelocity, desiredLinearAcceleration);
      spatialFeedbackControlCommand.set(desiredContactOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      updateSelectionMatrices();

      updateGraphics();
   }

   private void updateDesiredContactPose()
   {
      currentContactPosition.setToZero(contactFrame);
      currentContactOrientation.setToZero(contactFrame);
      currentContactPosition.changeFrame(worldFrame);
      currentContactOrientation.changeFrame(worldFrame);

      desiredContactPosition.checkReferenceFrameMatch(currentContactPosition);
      desiredContactOrientation.checkReferenceFrameMatch(currentContactOrientation);

      // TODO: fix this to work with arbitrary surface normals
      desiredContactPosition.setX(currentContactPosition.getX());
      desiredContactPosition.setY(currentContactPosition.getY());
      desiredContactPosition.setZ(currentContactPosition.getZ());

      desiredContactFrame.setPoseAndUpdate(desiredContactPosition, desiredContactOrientation);
   }

   private void updateSelectionMatrices()
   {
      // set selection matrices
      accelerationSelectionMatrix.reshape(dofs, dofs);
      CommonOps.setIdentity(accelerationSelectionMatrix);
      feedbackSelectionMatrix.reshape(dofs, dofs);
      CommonOps.setIdentity(feedbackSelectionMatrix);

      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;

      // TODO: figure out which directions to control based on support area
//      isDirectionFeedbackControlled[3] = true; // control x position
//      isDirectionFeedbackControlled[4] = true; // control y position
      isDirectionFeedbackControlled[2] = true; // control z orientation
      isDirectionFeedbackControlled[0] = true; // control x orientation
      isDirectionFeedbackControlled[1] = true; // control y orientation

      for (int i = dofs-1; i >= 0; i--)
      {
         if (isDirectionFeedbackControlled[i])
            MatrixTools.removeRow(accelerationSelectionMatrix, i);
         else
            MatrixTools.removeRow(feedbackSelectionMatrix, i);
      }

      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);
      if (accelerationSelectionMatrix.getNumRows() + feedbackSelectionMatrix.getNumRows() != dofs)
         throw new RuntimeException("Trying to control too much or too little.");
   }

   @Override
   public void doTransitionIntoAction()
   {
      desiredContactPosition.setToZero(contactFrame);
      desiredContactOrientation.setToZero(contactFrame);

      desiredContactPosition.changeFrame(worldFrame);
      desiredContactOrientation.changeFrame(worldFrame);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      hideGraphics();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(spatialAccelerationCommand);
      inverseDynamicsCommandList.addCommand(planeContactStateCommand);
      return inverseDynamicsCommandList;
   }

   public InverseDynamicsCommand<?> getEmptyPlaneContactStateCommand()
   {
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(body);
      planeContactStateCommand.setId(NO_CONTACT_ID);

      updateGraphics();
      return planeContactStateCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
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

   private void updateGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
      {
         graphics.get(graphicsIdx).showGraphicObject();
         graphics.get(graphicsIdx).update();
      }
   }

   private void hideGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
         graphics.get(graphicsIdx).hideGraphicObject();
   }

}
