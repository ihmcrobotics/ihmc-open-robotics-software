package us.ihmc.sensorProcessing.stateEstimation;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

//assumes that twist calculator and spatial acceleration calculator have already been updated with joint positions and velocities
//TODO: update accelerations
public class CenterOfMassStateFullRobotModelUpdater implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;

   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public CenterOfMassStateFullRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort, ControlFlowOutputPort<FrameOrientation> orientationPort,
         ControlFlowOutputPort<FrameVector> angularVelocityPort, ControlFlowOutputPort<FrameVector> angularAccelerationPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.orientationPort = orientationPort;

      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();

      RigidBody elevator = inverseDynamicsStructure.getElevator();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJoint.getFrameAfterJoint());
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator),
            ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()), rootJoint.getFrameAfterJoint());
   }

   public void run()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      centerOfMassCalculator.compute();

      updateRootJointPositionAndRotation(rootJoint, estimationFrame, centerOfMassPositionPort.getData());
      rootJoint.getFrameAfterJoint().update();

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      updateRootJointTwist(twistCalculator);
      twistCalculator.compute();
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector tempRootJointAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointLinearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void updateRootJointTwist(TwistCalculator twistCalculator)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      //TODO: check if frames are correct of tempRootJointAngularVelocity
      rootJoint.packJointTwist(tempRootJointTwist);
      tempRootJointTwist.packAngularPart(tempRootJointAngularVelocity);

      computeRootJointLinearVelocity(tempRootJointLinearVelocity, tempRootJointAngularVelocity);

      computeRootJointTwistLinearVelocityOnly(rootJoint, tempRootJointTwist, tempRootJointLinearVelocity);
      rootJoint.setJointTwist(tempRootJointTwist);

   }

   private final FramePoint tempComBody = new FramePoint();
   private final FrameVector tempComVelocityBody = new FrameVector();
   private final FrameVector tempCenterOfMassVelocityOffset = new FrameVector();
   private final FrameVector tempCrossPart = new FrameVector();
   private final FrameVector tempCenterOfMassVelocityWorld = new FrameVector(ReferenceFrame.getWorldFrame());

   private void computeRootJointLinearVelocity(FrameVector rootJointVelocityToPack, FrameVector rootJointAngularVelocity)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      tempCenterOfMassVelocityWorld.setAndChangeFrame(centerOfMassVelocityPort.getData());

      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();

      // \dot{r}^{root}
      centerOfMassJacobianBody.compute();
      tempComVelocityBody.setToZero(rootJointFrame);
      centerOfMassJacobianBody.packCenterOfMassVelocity(tempComVelocityBody);
      tempComVelocityBody.changeFrame(rootJointFrame);

      // \tilde{\omega} r^{root}
      tempComBody.setToZero(rootJointFrame);
      centerOfMassCalculator.packCenterOfMass(tempComBody);
      tempComBody.changeFrame(rootJointFrame);
      tempCrossPart.setToZero(rootJointFrame);
      tempCrossPart.cross(rootJointAngularVelocity, tempComBody);

      // v_{r/p}= \tilde{\omega} r^{root} + \dot{r}^{root}
      tempCenterOfMassVelocityOffset.setToZero(rootJointFrame);
      tempCenterOfMassVelocityOffset.add(tempCrossPart, tempComVelocityBody);

      // v_{root}^{p,w} = R_{w}^{root} \dot{r} - v_{r/p}
      tempCenterOfMassVelocityWorld.changeFrame(rootJointFrame);
      rootJointVelocityToPack.setAndChangeFrame(tempCenterOfMassVelocityWorld);
      rootJointVelocityToPack.sub(tempCenterOfMassVelocityOffset);

   }

   private final Twist tempRootJointTwistExisting = new Twist();
   private final FrameVector tempRootJointTwistExistingAngularPart = new FrameVector();

   private void computeRootJointTwistLinearVelocityOnly(SixDoFJoint rootJoint, Twist rootJointTwistToPack, FrameVector rootJointLinearVelocity)
   {
      rootJoint.packJointTwist(tempRootJointTwistExisting);
      tempRootJointTwistExisting.checkReferenceFramesMatch(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      tempRootJointTwistExisting.packAngularPart(tempRootJointTwistExistingAngularPart);

      rootJointLinearVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());

      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
            rootJointLinearVelocity.getVector(), tempRootJointTwistExistingAngularPart.getVector());
   }

   private final FramePoint tempCenterOfMassPositionState = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameOrientation tempOrientationState = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final Transform3D tempEstimationLinkToWorld = new Transform3D();
   private final Transform3D tempRootJointToWorld = new Transform3D();

   private void updateRootJointPositionAndRotation(SixDoFJoint rootJoint, ReferenceFrame estimationFrame, FramePoint centerOfMassPosition)
   {
      tempCenterOfMassPositionState.setAndChangeFrame(centerOfMassPosition);
      tempOrientationState.setAndChangeFrame(orientationPort.getData());

      computeEstimationLinkToWorldTransform(estimationFrame, tempEstimationLinkToWorld, tempCenterOfMassPositionState, tempOrientationState);
      computeRootJointToWorldTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      rootJoint.setPositionAndRotation(tempRootJointToWorld);
   }

   private final FramePoint tempCenterOfMassBody = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d tempCenterOfMassBodyVector3d = new Vector3d();
   private final Point3d tempEstimationLinkPosition = new Point3d();
   private final Vector3d tempEstimationLinkPositionVector3d = new Vector3d();

   private void computeEstimationLinkToWorldTransform(ReferenceFrame estimationFrame, Transform3D estimationLinkToWorldToPack, FramePoint centerOfMassWorld,
         FrameOrientation estimationLinkOrientation)
   {
      // r^{estimation}
      tempCenterOfMassBody.setToZero(estimationFrame);
      centerOfMassCalculator.packCenterOfMass(tempCenterOfMassBody);
      tempCenterOfMassBody.changeFrame(estimationFrame);

      // R_{estimation}^{w}
      estimationLinkOrientation.changeFrame(worldFrame);
      estimationLinkOrientation.getTransform3D(estimationLinkToWorldToPack);

      // R_{estimation}^{w} * r^{estimation}
      tempCenterOfMassBody.getVector(tempCenterOfMassBodyVector3d);
      estimationLinkToWorldToPack.transform(tempCenterOfMassBodyVector3d);

      // p_{estimation}^{w} = r^{w} - R_{estimation}^{w} r^{estimation}
      centerOfMassWorld.getPoint(tempEstimationLinkPosition);
      tempEstimationLinkPosition.sub(tempCenterOfMassBodyVector3d);

      // H_{estimation}^{w}
      tempEstimationLinkPositionVector3d.set(tempEstimationLinkPosition);
      estimationLinkToWorldToPack.setTranslation(tempEstimationLinkPositionVector3d);
   }

   private final Transform3D tempRootJointFrameToEstimationFrame = new Transform3D();

   private void computeRootJointToWorldTransform(SixDoFJoint rootJoint, ReferenceFrame estimationFrame, Transform3D rootJointToWorldToPack,
         Transform3D estimationLinkTransform)
   {
      // H_{root}^{estimation}
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(tempRootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      rootJointToWorldToPack.set(estimationLinkTransform);
      rootJointToWorldToPack.mul(tempRootJointFrameToEstimationFrame);
   }
}
