package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class PositionStateRobotModelUpdater implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionOutputPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityOutputPort;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public PositionStateRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           ControlFlowOutputPort<FramePoint> centerOfMassPositionOutputPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityOutputPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.centerOfMassPositionOutputPort = centerOfMassPositionOutputPort;
      this.centerOfMassVelocityOutputPort = centerOfMassVelocityOutputPort;

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

      updateRootJointPosition(rootJoint, estimationFrame, centerOfMassPositionOutputPort.getData());
      rootJoint.getFrameAfterJoint().update();

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      updateRootJointTwistLinearPart(centerOfMassVelocityOutputPort.getData(), rootJoint);
      twistCalculator.compute();
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector tempRootJointAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointLinearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void updateRootJointTwistLinearPart(FrameVector centerOfMassVelocityWorld, SixDoFJoint rootJoint)
   {
      rootJoint.packJointTwist(tempRootJointTwist);
      tempRootJointTwist.packAngularPart(tempRootJointAngularVelocity);

      computeRootJointLinearVelocity(centerOfMassVelocityWorld, tempRootJointLinearVelocity, tempRootJointAngularVelocity, rootJoint);
      computeRootJointTwistLinearPart(rootJoint, tempRootJointTwist, tempRootJointLinearVelocity);

      rootJoint.setJointTwist(tempRootJointTwist);

   }

   private final FramePoint tempComBody = new FramePoint();
   private final FrameVector tempComVelocityBody = new FrameVector();
   private final FrameVector tempCenterOfMassVelocityOffset = new FrameVector();
   private final FrameVector tempCrossPart = new FrameVector();
   private final FrameVector tempCenterOfMassVelocityWorld = new FrameVector(ReferenceFrame.getWorldFrame());

   private void computeRootJointLinearVelocity(FrameVector centerOfMassVelocityWorld, FrameVector rootJointVelocityToPack,
           FrameVector rootJointAngularVelocity, SixDoFJoint rootJoint)
   {
      tempCenterOfMassVelocityWorld.setIncludingFrame(centerOfMassVelocityWorld);
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
      rootJointVelocityToPack.setIncludingFrame(tempCenterOfMassVelocityWorld);
      rootJointVelocityToPack.sub(tempCenterOfMassVelocityOffset);
   }

   private final Twist tempRootJointTwistExisting = new Twist();
   private final FrameVector tempRootJointTwistExistingAngularPart = new FrameVector();

   private void computeRootJointTwistLinearPart(SixDoFJoint rootJoint, Twist rootJointTwistToPack, FrameVector rootJointLinearVelocity)
   {
      rootJoint.packJointTwist(tempRootJointTwistExisting);
      tempRootJointTwistExisting.checkReferenceFramesMatch(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      tempRootJointTwistExisting.packAngularPart(tempRootJointTwistExistingAngularPart);

      rootJointLinearVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());

      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                               rootJointLinearVelocity.getVector(), tempRootJointTwistExistingAngularPart.getVector());
   }

   private final FramePoint tempCenterOfMassPositionState = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Matrix3d tempOrientationStateReconstructMatrix = new Matrix3d();
   private final FrameOrientation tempOrientationStateReconstruct = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempEstimationLinkToWorld = new RigidBodyTransform();
   private final RigidBodyTransform tempRootJointToWorld = new RigidBodyTransform();

   private void updateRootJointPosition(SixDoFJoint rootJoint, ReferenceFrame estimationFrame, FramePoint centerOfMassPosition)
   {
      tempCenterOfMassPositionState.setIncludingFrame(centerOfMassPosition);
      estimationFrame.getTransformToDesiredFrame(worldFrame).get(tempOrientationStateReconstructMatrix);
      tempOrientationStateReconstruct.set(tempOrientationStateReconstructMatrix);

      computeEstimationLinkToWorldTransform(estimationFrame, tempEstimationLinkToWorld, tempCenterOfMassPositionState, tempOrientationStateReconstruct);
      computeRootJointToWorldTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      rootJoint.setPositionAndRotation(tempRootJointToWorld);
   }

   private final FramePoint tempCenterOfMassBody = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d tempCenterOfMassBodyVector3d = new Vector3d();
   private final Point3d tempEstimationLinkPosition = new Point3d();
   private final Vector3d tempEstimationLinkPositionVector3d = new Vector3d();

   private void computeEstimationLinkToWorldTransform(ReferenceFrame estimationFrame, RigidBodyTransform estimationLinkToWorldToPack, FramePoint centerOfMassWorld,
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

   private void computeRootJointToWorldTransform(SixDoFJoint rootJoint, ReferenceFrame estimationFrame, RigidBodyTransform rootJointToWorldToPack,
           RigidBodyTransform estimationLinkTransform)
   {
      // H_{root}^{estimation}
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(tempRootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      rootJointToWorldToPack.set(estimationLinkTransform);
      rootJointToWorldToPack.multiply(tempRootJointFrameToEstimationFrame);
   }


}
