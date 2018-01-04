package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class PositionStateRobotModelUpdater implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FramePoint3D> centerOfMassPositionOutputPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityOutputPort;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public PositionStateRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           ControlFlowOutputPort<FramePoint3D> centerOfMassPositionOutputPort, ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityOutputPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.centerOfMassPositionOutputPort = centerOfMassPositionOutputPort;
      this.centerOfMassVelocityOutputPort = centerOfMassVelocityOutputPort;

      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      RigidBody elevator = inverseDynamicsStructure.getElevator();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJoint.getFrameAfterJoint());
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator),
              ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()), rootJoint.getFrameAfterJoint());
   }

   public void run()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      centerOfMassCalculator.compute();

      updateRootJointPosition(rootJoint, estimationFrame, centerOfMassPositionOutputPort.getData());
      rootJoint.getFrameAfterJoint().update();

      updateRootJointTwistLinearPart(centerOfMassVelocityOutputPort.getData(), rootJoint);
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector3D tempRootJointAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempRootJointLinearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void updateRootJointTwistLinearPart(FrameVector3D centerOfMassVelocityWorld, FloatingInverseDynamicsJoint rootJoint)
   {
      rootJoint.getJointTwist(tempRootJointTwist);
      tempRootJointTwist.getAngularPart(tempRootJointAngularVelocity);

      computeRootJointLinearVelocity(centerOfMassVelocityWorld, tempRootJointLinearVelocity, tempRootJointAngularVelocity, rootJoint);
      computeRootJointTwistLinearPart(rootJoint, tempRootJointTwist, tempRootJointLinearVelocity);

      rootJoint.setJointTwist(tempRootJointTwist);

   }

   private final FramePoint3D tempComBody = new FramePoint3D();
   private final FrameVector3D tempComVelocityBody = new FrameVector3D();
   private final FrameVector3D tempCenterOfMassVelocityOffset = new FrameVector3D();
   private final FrameVector3D tempCrossPart = new FrameVector3D();
   private final FrameVector3D tempCenterOfMassVelocityWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void computeRootJointLinearVelocity(FrameVector3D centerOfMassVelocityWorld, FrameVector3D rootJointVelocityToPack,
           FrameVector3D rootJointAngularVelocity, FloatingInverseDynamicsJoint rootJoint)
   {
      tempCenterOfMassVelocityWorld.setIncludingFrame(centerOfMassVelocityWorld);
      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();

      // \dot{r}^{root}
      centerOfMassJacobianBody.compute();
      tempComVelocityBody.setToZero(rootJointFrame);
      centerOfMassJacobianBody.getCenterOfMassVelocity(tempComVelocityBody);
      tempComVelocityBody.changeFrame(rootJointFrame);

      // \tilde{\omega} r^{root}
      tempComBody.setToZero(rootJointFrame);
      centerOfMassCalculator.getCenterOfMass(tempComBody);
      tempComBody.changeFrame(rootJointFrame);
      tempCrossPart.setToZero(rootJointFrame);
      tempCrossPart.cross(rootJointAngularVelocity, tempComBody);

      // v_{r/p}= \tilde{\omega} r^{root} + \dot{r}^{root}
      tempCenterOfMassVelocityOffset.setToZero(rootJointFrame);
      tempCenterOfMassVelocityOffset.add(tempCrossPart, tempComVelocityBody);

      // v_{root}^{p,w} = R_{w}^{root} \dot{r} - v_{r/p}
      tempCenterOfMassVelocityWorld.changeFrame(rootJointFrame);
      rootJointVelocityToPack.setIncludingFrame(tempCenterOfMassVelocityWorld);
      rootJointVelocityToPack.sub(tempCenterOfMassVelocityOffset);
   }

   private final Twist tempRootJointTwistExisting = new Twist();
   private final FrameVector3D tempRootJointTwistExistingAngularPart = new FrameVector3D();

   private void computeRootJointTwistLinearPart(FloatingInverseDynamicsJoint rootJoint, Twist rootJointTwistToPack, FrameVector3D rootJointLinearVelocity)
   {
      rootJoint.getJointTwist(tempRootJointTwistExisting);
      tempRootJointTwistExisting.checkReferenceFramesMatch(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      tempRootJointTwistExisting.getAngularPart(tempRootJointTwistExistingAngularPart);

      rootJointLinearVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());

      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                               rootJointLinearVelocity.getVector(), tempRootJointTwistExistingAngularPart.getVector());
   }

   private final FramePoint3D tempCenterOfMassPositionState = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final RotationMatrix tempOrientationStateReconstructMatrix = new RotationMatrix();
   private final FrameQuaternion tempOrientationStateReconstruct = new FrameQuaternion(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempEstimationLinkToWorld = new RigidBodyTransform();
   private final RigidBodyTransform tempRootJointToWorld = new RigidBodyTransform();

   private void updateRootJointPosition(FloatingInverseDynamicsJoint rootJoint, ReferenceFrame estimationFrame, FramePoint3D centerOfMassPosition)
   {
      tempCenterOfMassPositionState.setIncludingFrame(centerOfMassPosition);
      estimationFrame.getTransformToDesiredFrame(worldFrame).getRotation(tempOrientationStateReconstructMatrix);
      tempOrientationStateReconstruct.set(tempOrientationStateReconstructMatrix);

      computeEstimationLinkToWorldTransform(estimationFrame, tempEstimationLinkToWorld, tempCenterOfMassPositionState, tempOrientationStateReconstruct);
      computeRootJointToWorldTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      rootJoint.setPositionAndRotation(tempRootJointToWorld);
   }

   private final FramePoint3D tempCenterOfMassBody = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final Vector3D tempCenterOfMassBodyVector3d = new Vector3D();
   private final Point3D tempEstimationLinkPosition = new Point3D();
   private final Vector3D tempEstimationLinkPositionVector3d = new Vector3D();

   private void computeEstimationLinkToWorldTransform(ReferenceFrame estimationFrame, RigidBodyTransform estimationLinkToWorldToPack, FramePoint3D centerOfMassWorld,
           FrameQuaternion estimationLinkOrientation)
   {
      // r^{estimation}
      tempCenterOfMassBody.setToZero(estimationFrame);
      centerOfMassCalculator.getCenterOfMass(tempCenterOfMassBody);
      tempCenterOfMassBody.changeFrame(estimationFrame);

      // R_{estimation}^{w}
      estimationLinkOrientation.changeFrame(worldFrame);
      estimationLinkToWorldToPack.setRotation(estimationLinkOrientation);

      // R_{estimation}^{w} * r^{estimation}
      tempCenterOfMassBody.get(tempCenterOfMassBodyVector3d);
      estimationLinkToWorldToPack.transform(tempCenterOfMassBodyVector3d);

      // p_{estimation}^{w} = r^{w} - R_{estimation}^{w} r^{estimation}
      centerOfMassWorld.get(tempEstimationLinkPosition);
      tempEstimationLinkPosition.sub(tempCenterOfMassBodyVector3d);

      // H_{estimation}^{w}
      tempEstimationLinkPositionVector3d.set(tempEstimationLinkPosition);
      estimationLinkToWorldToPack.setTranslation(tempEstimationLinkPositionVector3d);
   }

   private final RigidBodyTransform tempRootJointFrameToEstimationFrame = new RigidBodyTransform();

   private void computeRootJointToWorldTransform(FloatingInverseDynamicsJoint rootJoint, ReferenceFrame estimationFrame, RigidBodyTransform rootJointToWorldToPack,
           RigidBodyTransform estimationLinkTransform)
   {
      // H_{root}^{estimation}
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(tempRootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      rootJointToWorldToPack.set(estimationLinkTransform);
      rootJointToWorldToPack.multiply(tempRootJointFrameToEstimationFrame);
   }


}
