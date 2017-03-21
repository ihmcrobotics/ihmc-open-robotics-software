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
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
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
   private static final FrameVector zeroInWorld = new FrameVector(worldFrame, 0.0, 0.0, 0.0);

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();

   private final DenseMatrix64F accelerationSelectionMatrix = new DenseMatrix64F(dofs, dofs);
   private final DenseMatrix64F feedbackSelectionMatrix = new DenseMatrix64F(dofs, dofs);
   private final boolean[] isDirectionFeedbackControlled = new boolean[dofs];

   private final FramePose bodyFixedControlledPose = new FramePose();
   private final SpatialAccelerationVector bodyAcceleration;

   // TODO: allow multiple contact points
   private final YoFramePoint contactPoint;
   private final YoFramePoint contactPointInWorld;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame elevatorFrame;
   private final PoseReferenceFrame desiredContactFrame;

   private final ContactablePlaneBody contactableBody;
   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormal;
   private final ReferenceFrame contactFrame;

   private final RigidBodyTransform bodyToJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform contactToJointTransform = new RigidBodyTransform();

   private final FramePoint desiredContactPosition = new FramePoint(worldFrame);
   private final FrameOrientation desiredContactOrientation = new FrameOrientation(worldFrame);
   private final FramePoint currentContactPosition = new FramePoint(worldFrame);
   private final FrameOrientation currentContactOrientation = new FrameOrientation(worldFrame);

   private final ArrayList<YoGraphic> graphics = new ArrayList<>();

   public RigidBodyLoadBearingControlState(RigidBody bodyToControl, ContactablePlaneBody contactableBody, RigidBody elevator, DoubleYoVariable yoTime,
         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOADBEARING, bodyToControl.getName(), yoTime, parentRegistry);
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.elevatorFrame = elevator.getBodyFixedFrame();
      this.contactFrame = contactableBody.getSoleFrame();
      this.contactableBody = contactableBody;

      bodyFrame.getTransformToDesiredFrame(bodyToJointTransform, bodyToControl.getParentJoint().getFrameAfterJoint());

      bodyAcceleration = new SpatialAccelerationVector(contactFrame, elevatorFrame, contactFrame);
      spatialAccelerationCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.set(elevator, bodyToControl);

      String bodyName = bodyToControl.getName();
      coefficientOfFriction = new DoubleYoVariable(bodyName + "CoefficientOfFriction", registry);
      contactNormal = new YoFrameVector(bodyName + "ContactNormal", worldFrame, parentRegistry);
      contactPoint = new YoFramePoint(bodyName + "ContactPoint", contactFrame, parentRegistry);
      contactPointInWorld = new YoFramePoint(bodyName + "ContactPointInWorld", worldFrame, parentRegistry);
      desiredContactFrame = new PoseReferenceFrame(bodyName + "DesiredContactFrame", worldFrame);

      planeContactStateCommand.setContactingRigidBody(bodyToControl);
      planeContactStateCommand.setId(NO_CONTACT_ID);

      setupViz(graphicsListRegistry, bodyName);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();

//      YoGraphicReferenceFrame contactFrameViz = new YoGraphicReferenceFrame(contactFrame, registry, 0.14);
//      graphicsListRegistry.registerYoGraphic(listName, contactFrameViz);
//      graphics.add(contactFrameViz);

//      YoGraphicReferenceFrame desiredContactFrameViz = new YoGraphicReferenceFrame(desiredContactFrame, registry, 0.07);
//      graphicsListRegistry.registerYoGraphic(listName, desiredContactFrameViz);
//      graphics.add(desiredContactFrameViz);

      YoGraphicVector surfaceNormal = new YoGraphicVector(bodyName + "ContactNormal", contactPointInWorld, contactNormal, 0.1, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, surfaceNormal);
      graphics.add(surfaceNormal);

      YoGraphicPosition contactPoint = new YoGraphicPosition(bodyName + "ContactPoint", contactPointInWorld, 0.01, YoAppearance.Black());
      graphicsListRegistry.registerYoGraphic(listName, contactPoint);
      graphics.add(contactPoint);
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

   public void setAndUpdateContactFrame(RigidBodyTransform bodyToContactFrame)
   {
      contactToJointTransform.set(bodyToJointTransform);
      contactToJointTransform.multiply(bodyToContactFrame);
      contactableBody.setSoleFrameTransformFromParentJoint(contactToJointTransform);
      contactPoint.setToZero();
   }

   @Override
   public void doAction()
   {
      updateInternal();

      // assemble contact command
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommand.setContactNormal(contactNormal.getFrameTuple());
      planeContactStateCommand.addPointInContact(contactPoint.getFrameTuple());
      planeContactStateCommand.setId(IN_CONTACT_ID);

      // assemble zero acceleration command
      bodyAcceleration.setToZero(contactFrame, elevatorFrame, contactFrame);
      bodyAcceleration.changeBodyFrameNoRelativeAcceleration(bodyFrame);
      bodyAcceleration.changeFrameNoRelativeMotion(bodyFrame);
      spatialAccelerationCommand.setSpatialAcceleration(bodyAcceleration);
      spatialAccelerationCommand.setSelectionMatrix(accelerationSelectionMatrix);

      // assemble feedback control command
      bodyFixedControlledPose.setToZero(contactFrame);
      bodyFixedControlledPose.changeFrame(bodyFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyFixedControlledPose);
      spatialFeedbackControlCommand.set(desiredContactPosition, zeroInWorld, zeroInWorld);
      spatialFeedbackControlCommand.set(desiredContactOrientation, zeroInWorld, zeroInWorld);
      spatialFeedbackControlCommand.setSelectionMatrix(feedbackSelectionMatrix);

      updateGraphics();
   }

   private void updateInternal()
   {
      // update current contact information
      currentContactPosition.setToZero(contactFrame);
      currentContactOrientation.setToZero(contactFrame);
      currentContactPosition.changeFrame(desiredContactPosition.getReferenceFrame());
      currentContactOrientation.changeFrame(desiredContactOrientation.getReferenceFrame());

      // TODO: figure out which directions to control based on support area
      // This requires the selection matrix in the command to be in contact frame.
      // For now we just hold the orientation and not the position of the contact point in world frame
      for (int i = 0; i < dofs; i++)
         isDirectionFeedbackControlled[i] = false;
      isDirectionFeedbackControlled[0] = true; // control x orientation
      isDirectionFeedbackControlled[1] = true; // control y orientation
      isDirectionFeedbackControlled[2] = true; // control z orientation
      desiredContactPosition.setX(currentContactPosition.getX()); // do not control x position
      desiredContactPosition.setY(currentContactPosition.getY()); // do not control y position
      desiredContactPosition.setZ(currentContactPosition.getZ()); // do not control z position

      // update things for visualization
      desiredContactPosition.checkReferenceFrameMatch(desiredContactFrame.getParent());
      desiredContactOrientation.checkReferenceFrameMatch(desiredContactFrame.getParent());
      desiredContactFrame.setPoseAndUpdate(desiredContactPosition, desiredContactOrientation);
      contactPointInWorld.setAndMatchFrame(contactPoint);

      // assemble the selection matrices for the controller core commands
      accelerationSelectionMatrix.reshape(dofs, dofs);
      CommonOps.setIdentity(accelerationSelectionMatrix);
      feedbackSelectionMatrix.reshape(dofs, dofs);
      CommonOps.setIdentity(feedbackSelectionMatrix);

      for (int i = dofs-1; i >= 0; i--)
      {
         if (isDirectionFeedbackControlled[i])
            MatrixTools.removeRow(accelerationSelectionMatrix, i);
         else
            MatrixTools.removeRow(feedbackSelectionMatrix, i);
      }

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
      planeContactStateCommand.setId(NO_CONTACT_ID);
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
         graphics.get(graphicsIdx).update();
   }

   private void hideGraphics()
   {
      for (int graphicsIdx = 0; graphicsIdx < graphics.size(); graphicsIdx++)
      {
         YoGraphic yoGraphic = graphics.get(graphicsIdx);
         yoGraphic.hideGraphicObject();
      }
   }

}
