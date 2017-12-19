package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

//assumes that twist calculator and spatial acceleration calculator have already been updated with joint positions and velocities
//TODO: update accelerations
public class OrientationAndPositionFullRobotModelUpdater implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort;

   private final ControlFlowOutputPort<FrameQuaternion> orientationPort;
   private final ControlFlowOutputPort<FrameVector3D> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector3D> angularAccelerationPort;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;
   private final CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();


   public OrientationAndPositionFullRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort,
           ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort, ControlFlowOutputPort<FrameQuaternion> orientationPort,
           ControlFlowOutputPort<FrameVector3D> angularVelocityPort, ControlFlowOutputPort<FrameVector3D> angularAccelerationPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;

      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;

      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();

      RigidBody elevator = inverseDynamicsStructure.getElevator();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJoint.getFrameAfterJoint());
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(elevator),
              ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()), rootJoint.getFrameAfterJoint());

      // TODO: Should pass the input port for the spatial acceleration calculator here too...
      this.centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(rootJoint.getSuccessor(),
              ScrewTools.computeSupportAndSubtreeSuccessors(elevator), inverseDynamicsStructure.getSpatialAccelerationCalculator());
   }

   public void run()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      centerOfMassCalculator.compute();

      updateRootJointConfiguration(rootJoint, estimationFrame);
      rootJoint.getFrameAfterJoint().update();

      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      updateRootJointTwistAndSpatialAcceleration(spatialAccelerationCalculator);
      spatialAccelerationCalculator.compute();
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector3D tempRootJointAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempRootJointAngularAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempRootJointLinearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempRootJointLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final SpatialAccelerationVector tempRootJointAcceleration = new SpatialAccelerationVector();

   private void updateRootJointTwistAndSpatialAcceleration(SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      computeRootJointAngularVelocityAndAcceleration(spatialAccelerationCalculator, tempRootJointAngularVelocity,
              tempRootJointAngularAcceleration);
      computeRootJointLinearVelocityAndAcceleration(tempRootJointLinearVelocity, tempRootJointLinearAcceleration, tempRootJointAngularVelocity,
              tempRootJointAngularAcceleration);

      computeRootJointTwist(rootJoint, tempRootJointTwist, tempRootJointAngularVelocity, tempRootJointLinearVelocity);
      rootJoint.setJointTwist(tempRootJointTwist);
      rootJoint.updateFramesRecursively();
      computeRootJointAcceleration(rootJoint, tempRootJointAcceleration, tempRootJointAngularAcceleration, tempRootJointLinearAcceleration);
      rootJoint.setAcceleration(tempRootJointAcceleration);
   }

   private final Twist tempRootToEstimationTwist = new Twist();
   private final SpatialAccelerationVector tempRootToEstimationAcceleration = new SpatialAccelerationVector();
   private final FrameVector3D tempRootToEstimationAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempRootToEstimationAngularAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempCrossTerm = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempEstimationLinkAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void computeRootJointAngularVelocityAndAcceleration(SpatialAccelerationCalculator spatialAccelerationCalculator,
           FrameVector3D rootJointAngularVelocityToPack, FrameVector3D rootJointAngularAccelerationToPack)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();

      tempEstimationLinkAngularVelocity.setIncludingFrame(angularVelocityPort.getData());

      // T_{root}^{root, estimation}
      rootJoint.getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(estimationLink.getBodyFixedFrame(), tempRootToEstimationTwist);
      tempRootToEstimationTwist.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, estimation}
      tempRootToEstimationAngularVelocity.setToZero(rootJoint.getFrameAfterJoint());
      tempRootToEstimationTwist.getAngularPart(tempRootToEstimationAngularVelocity);

      // omega_{estimation}^{root, world}
      tempEstimationLinkAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, world} = omega_{estimation}^{root, world} + omega_{root}^{root, estimation}
      rootJointAngularVelocityToPack.setToZero(rootJoint.getFrameAfterJoint());
      rootJointAngularVelocityToPack.add(tempEstimationLinkAngularVelocity, tempRootToEstimationAngularVelocity);

      // R_{estimation}^{root} ( \omega_{estimation}^{estimation, root} \times \omega_{estimation}^{estimation, world} )
      tempRootToEstimationAngularVelocity.negate();
      tempRootToEstimationAngularVelocity.changeFrame(estimationFrame);
      tempEstimationLinkAngularVelocity.changeFrame(estimationFrame);
      tempCrossTerm.setToZero(estimationFrame);
      tempCrossTerm.cross(tempRootToEstimationAngularVelocity, tempEstimationLinkAngularVelocity);

      // \omega_{root}^{root, estimation}
      spatialAccelerationCalculator.getRelativeAcceleration(estimationLink, rootJoint.getSuccessor(), tempRootToEstimationAcceleration);
      tempRootToEstimationAcceleration.changeFrameNoRelativeMotion(rootJoint.getFrameAfterJoint());
      tempRootToEstimationAcceleration.getAngularPart(tempRootToEstimationAngularAcceleration);
      tempRootToEstimationAngularAcceleration.changeFrame(estimationFrame);

      rootJointAngularAccelerationToPack.setIncludingFrame(angularAccelerationPort.getData());
      rootJointAngularAccelerationToPack.add(tempCrossTerm);
      rootJointAngularAccelerationToPack.add(tempRootToEstimationAngularAcceleration);
      rootJointAngularAccelerationToPack.changeFrame(rootJoint.getFrameAfterJoint());
   }

   private final FramePoint3D tempComBody = new FramePoint3D();
   private final FrameVector3D tempComVelocityBody = new FrameVector3D();
   private final FrameVector3D tempComAccelerationBody = new FrameVector3D();
   private final FrameVector3D tempCenterOfMassVelocityOffset = new FrameVector3D();
   private final FrameVector3D tempCrossPart = new FrameVector3D();
   private final FrameVector3D tempAngularAcceleration = new FrameVector3D();
   private final FrameVector3D tempCenterOfMassVelocityWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void computeRootJointLinearVelocityAndAcceleration(FrameVector3D rootJointVelocityToPack, FrameVector3D rootJointAccelerationToPack,
           FrameVector3D rootJointAngularVelocity, FrameVector3D rootJointAngularAcceleration)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      tempCenterOfMassVelocityWorld.setIncludingFrame(centerOfMassVelocityPort.getData());

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

      // R_{w}^{p} \ddot{r}
      rootJointAccelerationToPack.setIncludingFrame(centerOfMassAccelerationPort.getData());
      rootJointAccelerationToPack.changeFrame(rootJointFrame);

      // -\tilde{\omega} R_{w}^{p} \dot{r}
      tempCrossPart.setToZero(rootJointFrame);
      tempCrossPart.cross(rootJointAngularVelocity, tempCenterOfMassVelocityWorld);
      rootJointAccelerationToPack.sub(tempCrossPart);

      // -\tilde{\dot{\omega}}r^{p}
      tempAngularAcceleration.setIncludingFrame(rootJointAngularAcceleration);
      tempAngularAcceleration.changeFrame(rootJointFrame);
      tempCrossPart.cross(tempAngularAcceleration, tempComBody);
      rootJointAccelerationToPack.sub(tempCrossPart);

      // -\tilde{\omega} \dot{r}^{p}
      tempCrossPart.cross(rootJointAngularVelocity, tempComVelocityBody);
      rootJointAccelerationToPack.sub(tempCrossPart);

      // -\ddot{r}^{p}
      centerOfMassAccelerationCalculator.getCoMAcceleration(tempComAccelerationBody);
      tempComAccelerationBody.changeFrame(rootJointFrame);
      rootJointAccelerationToPack.sub(tempComAccelerationBody);
   }

   private void computeRootJointTwist(FloatingInverseDynamicsJoint rootJoint, Twist rootJointTwistToPack, FrameVector3D rootJointAngularVelocity,
                                      FrameVector3D rootJointLinearVelocity)
   {
      rootJointAngularVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointLinearVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                               rootJointLinearVelocity.getVector(), rootJointAngularVelocity.getVector());
   }

   private void computeRootJointAcceleration(FloatingInverseDynamicsJoint rootJoint, SpatialAccelerationVector rootJointAcceleration, FrameVector3D rootJointAngularAcceleration,
           FrameVector3D rootJointLinearAcceleration)
   {
      rootJointAngularAcceleration.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointLinearAcceleration.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointAcceleration.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                                rootJointLinearAcceleration.getVector(), rootJointAngularAcceleration.getVector());
   }

   private final FramePoint3D tempCenterOfMassPositionState = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FrameQuaternion tempOrientationState = new FrameQuaternion(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempEstimationLinkToWorld = new RigidBodyTransform();
   private final RigidBodyTransform tempRootJointToWorld = new RigidBodyTransform();

   private void updateRootJointConfiguration(FloatingInverseDynamicsJoint rootJoint, ReferenceFrame estimationFrame)
   {
      tempCenterOfMassPositionState.setIncludingFrame(centerOfMassPositionPort.getData());
      tempOrientationState.setIncludingFrame(orientationPort.getData());

      computeEstimationLinkTransform(estimationFrame, tempEstimationLinkToWorld, tempCenterOfMassPositionState, tempOrientationState);
      computeRootJointTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      rootJoint.setPositionAndRotation(tempRootJointToWorld);
   }

   private final FramePoint3D tempCenterOfMassBody = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final Vector3D tempCenterOfMassBodyVector3d = new Vector3D();
   private final Point3D tempEstimationLinkPosition = new Point3D();
   private final Vector3D tempEstimationLinkPositionVector3d = new Vector3D();

   private void computeEstimationLinkTransform(ReferenceFrame estimationFrame, RigidBodyTransform estimationLinkToWorldToPack, FramePoint3D centerOfMassWorld,
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

   private void computeRootJointTransform(FloatingInverseDynamicsJoint rootJoint, ReferenceFrame estimationFrame, RigidBodyTransform rootJointToWorldToPack,
           RigidBodyTransform estimationLinkTransform)
   {
      // H_{root}^{estimation}
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(tempRootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      rootJointToWorldToPack.set(estimationLinkTransform);
      rootJointToWorldToPack.multiply(tempRootJointFrameToEstimationFrame);
   }
}
