package us.ihmc.commonWalkingControlModules.stateEstimation;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

//assumes that twist calculator and spatial acceleration calculator have already been updated with joint positions and velocities
//TODO: update accelerations
public class CenterOfMassBasedFullRobotModelUpdater implements Runnable
{
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort;

   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector> angularAccelerationPort;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SixDoFJoint rootJoint;
   private final RigidBody estimationLink;

   private final SpatialAccelerationVector jointAcceleration;

   public CenterOfMassBasedFullRobotModelUpdater(TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator,
           ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
           ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort, ControlFlowOutputPort<FrameOrientation> orientationPort,
           ControlFlowOutputPort<FrameVector> angularVelocityPort, ControlFlowOutputPort<FrameVector> angularAccelerationPort, RigidBody estimationLink,
           ReferenceFrame estimationFrame, SixDoFJoint rootJoint)
   {
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;

      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;

      this.estimationLink = estimationLink;
      this.estimationFrame = estimationFrame;

      RigidBody elevator = rootJoint.getPredecessor();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJoint.getFrameAfterJoint());
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeRigidBodiesInOrder(elevator),
              ScrewTools.computeJointsInOrder(rootJoint.getSuccessor()), rootJoint.getFrameAfterJoint());
      this.rootJoint = rootJoint;

      this.jointAcceleration = new SpatialAccelerationVector(rootJoint.getFrameAfterJoint(), rootJoint.getPredecessor().getBodyFixedFrame(),
              rootJoint.getFrameAfterJoint());
   }

   public void run()
   {
      centerOfMassCalculator.compute();

      updateRootJointConfiguration();
      rootJoint.getFrameAfterJoint().update();

      updateRootJointTwistAndSpatialAcceleration();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector tempCenterOfMassVelocityWorld = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempEstimationLinkAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointLinearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootJointLinearAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final SpatialAccelerationVector tempRootJointAcceleration = new SpatialAccelerationVector();

   private void updateRootJointTwistAndSpatialAcceleration()
   {
      tempCenterOfMassVelocityWorld.setAndChangeFrame(centerOfMassVelocityPort.getData());
      tempEstimationLinkAngularVelocity.setAndChangeFrame(angularVelocityPort.getData());

      computeRootJointAngularVelocityAndAcceleration(tempRootJointAngularVelocity, tempRootJointAngularAcceleration, tempEstimationLinkAngularVelocity);
      computeRootJointLinearVelocity(tempRootJointLinearVelocity, tempCenterOfMassVelocityWorld, tempRootJointAngularVelocity);

      computeRootJointTwist(tempRootJointTwist, tempRootJointAngularVelocity, tempRootJointLinearVelocity);
      rootJoint.setJointTwist(tempRootJointTwist);

      tempRootJointLinearAcceleration.setToZero(rootJoint.getFrameAfterJoint()); // TODO
      computeRootJointAcceleration(tempRootJointAcceleration, tempRootJointAngularAcceleration, tempRootJointLinearAcceleration);
      rootJoint.setAcceleration(tempRootJointAcceleration);
   }

   private final Twist tempRootToEstimationTwist = new Twist();
   private final SpatialAccelerationVector tempRootToEstimationAcceleration = new SpatialAccelerationVector();
   private final FrameVector tempRootToEstimationAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempRootToEstimationAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempCrossTerm = new FrameVector(ReferenceFrame.getWorldFrame());

   private void computeRootJointAngularVelocityAndAcceleration(FrameVector rootJointAngularVelocityToPack, FrameVector rootJointAngularAccelerationToPack, FrameVector estimationLinkAngularVelocity)
   {
      // T_{root}^{root, estimation}
      twistCalculator.packRelativeTwist(tempRootToEstimationTwist, estimationLink, rootJoint.getSuccessor());
      tempRootToEstimationTwist.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, estimation}
      tempRootToEstimationAngularVelocity.setToZero(rootJoint.getFrameAfterJoint());
      tempRootToEstimationTwist.packAngularPart(tempRootToEstimationAngularVelocity);

      // omega_{estimation}^{root, world}
      estimationLinkAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, world} = omega_{estimation}^{root, world} + omega_{root}^{root, estimation}
      rootJointAngularVelocityToPack.setToZero(rootJoint.getFrameAfterJoint());
      rootJointAngularVelocityToPack.add(estimationLinkAngularVelocity, tempRootToEstimationAngularVelocity);
      
      // R_{estimation}^{root} ( \omega_{estimation}^{estimation, root} \times \omega_{estimation}^{estimation, world} ) 
      tempRootToEstimationAngularVelocity.changeFrame(estimationFrame);
      estimationLinkAngularVelocity.changeFrame(estimationFrame);
      tempCrossTerm.setToZero(estimationFrame);
      tempCrossTerm.cross(tempRootToEstimationAngularVelocity, estimationLinkAngularVelocity);
      
      // \omega_{root}^{root, estimation}
      spatialAccelerationCalculator.packRelativeAcceleration(tempRootToEstimationAcceleration, rootJoint.getSuccessor(), estimationLink);
      tempRootToEstimationAcceleration.packAngularPart(tempRootToEstimationAngularAcceleration);
      tempRootToEstimationAngularAcceleration.changeFrame(rootJoint.getFrameAfterJoint());
      
      rootJointAngularAccelerationToPack.setAndChangeFrame(angularAccelerationPort.getData());
      rootJointAngularAccelerationToPack.sub(tempCrossTerm);
      rootJointAngularAccelerationToPack.changeFrame(rootJoint.getFrameAfterJoint());
      rootJointAngularAccelerationToPack.add(tempRootToEstimationAngularAcceleration);
   }

   private final FrameVector tempComVelocityBody = new FrameVector();
   private final FramePoint tempComBody = new FramePoint();
   private final FrameVector tempCenterOfMassVelocityOffset = new FrameVector();
   private final FrameVector tempCrossPart = new FrameVector();

   private void computeRootJointLinearVelocity(FrameVector rootJointVelocityToPack, FrameVector centerOfMassVelocityWorld, FrameVector rootJointAngularVelocity)
   {
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
      rootJointVelocityToPack.setAndChangeFrame(centerOfMassVelocityWorld);
      rootJointVelocityToPack.changeFrame(rootJointFrame);
      rootJointVelocityToPack.sub(tempCenterOfMassVelocityOffset);
   }

   private void computeRootJointTwist(Twist rootJointTwistToPack, FrameVector rootJointAngularVelocity, FrameVector rootJointLinearVelocity)
   {
      rootJointAngularVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointLinearVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
                               rootJointLinearVelocity.getVector(), rootJointAngularVelocity.getVector());
   }

   private void computeRootJointAcceleration(SpatialAccelerationVector rootJointAcceleration, FrameVector rootJointAngularAcceleration,
         FrameVector rootJointLinearAcceleration)
   {
      rootJointAngularAcceleration.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointLinearAcceleration.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
      rootJointAcceleration.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
            rootJointLinearAcceleration.getVector(), rootJointAngularAcceleration.getVector());      
   }
   
   private final FramePoint tempCenterOfMassPositionState = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameOrientation tempOrientationState = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final Transform3D tempEstimationLinkToWorld = new Transform3D();
   private final Transform3D tempRootJointToWorld = new Transform3D();

   private void updateRootJointConfiguration()
   {
      tempCenterOfMassPositionState.setAndChangeFrame(centerOfMassPositionPort.getData());
      tempOrientationState.setAndChangeFrame(orientationPort.getData());

      computeEstimationLinkTransform(tempEstimationLinkToWorld, tempCenterOfMassPositionState, tempOrientationState);
      computeRootJointTransform(tempRootJointToWorld, tempEstimationLinkToWorld);
      rootJoint.setPositionAndRotation(tempRootJointToWorld);
   }

   private final FramePoint tempCenterOfMassBody = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d tempCenterOfMassBodyVector3d = new Vector3d();
   private final Point3d tempEstimationLinkPosition = new Point3d();
   private final Vector3d tempEstimationLinkPositionVector3d = new Vector3d();

   private void computeEstimationLinkTransform(Transform3D estimationLinkToWorldToPack, FramePoint centerOfMassWorld,
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

   private void computeRootJointTransform(Transform3D rootJointToWorldToPack, Transform3D estimationLinkTransform)
   {
      // H_{root}^{estimation}
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(tempRootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      rootJointToWorldToPack.set(estimationLinkTransform);
      rootJointToWorldToPack.mul(tempRootJointFrameToEstimationFrame);
   }
}
