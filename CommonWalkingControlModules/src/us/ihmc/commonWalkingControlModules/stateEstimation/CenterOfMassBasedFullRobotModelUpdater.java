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
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

// TODO: efficiency
// currently assumes that twist calculator has already been updated with joint positions and velocities
public class CenterOfMassBasedFullRobotModelUpdater implements Runnable
{
   private final TwistCalculator twistCalculator;
   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianBody;
   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SixDoFJoint rootJoint;
   private final RigidBody estimationLink;

   public CenterOfMassBasedFullRobotModelUpdater(TwistCalculator twistCalculator, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
                                ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, ControlFlowOutputPort<FrameOrientation> orientationPort,
                                ControlFlowOutputPort<FrameVector> angularVelocityPort, RigidBody estimationLink, ReferenceFrame estimationFrame,
                                SixDoFJoint rootJoint)
   {
      this.twistCalculator = twistCalculator;
      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.estimationLink = estimationLink;
      this.estimationFrame = estimationFrame;

      RigidBody elevator = rootJoint.getPredecessor();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, estimationFrame);
      this.centerOfMassJacobianBody = new CenterOfMassJacobian(ScrewTools.computeRigidBodiesInOrder(elevator),
              ScrewTools.computeJointsInOrder(rootJoint.getSuccessor()), estimationFrame);
      this.rootJoint = rootJoint;
   }

   public void run()
   {
      updateRootJointConfiguration();
      updateRootJointTwist();
      twistCalculator.compute();
   }

   private void updateRootJointTwist()
   {
      FrameVector centerOfMassVelocityWorld = centerOfMassVelocityPort.getData();
      FrameVector estimationLinkAngularVelocity = angularVelocityPort.getData();

      FrameVector estimationLinkLinearVelocity = computeEstimationLinkLinearVelocity(centerOfMassVelocityWorld, estimationLinkAngularVelocity);
      Twist estimationLinkTwist = computeEstimationLinkTwist(estimationLinkAngularVelocity, estimationLinkLinearVelocity);
      Twist rootJointTwist = computeRootJointTwist(estimationLinkTwist);
      rootJoint.setJointTwist(rootJointTwist);
   }

   private FrameVector computeEstimationLinkLinearVelocity(FrameVector centerOfMassVelocityWorld, FrameVector angularVelocityOfEstimationLink)
   {
      // \dot{r}^{estimation}
      FrameVector comVelocityBody = new FrameVector(estimationFrame);
      centerOfMassJacobianBody.packCenterOfMassVelocity(comVelocityBody);
      comVelocityBody.changeFrame(rootJoint.getFrameAfterJoint());

      // \tilde{\omega} r^{estimation}
      centerOfMassCalculator.compute();
      FramePoint centerOfMassBody = new FramePoint(estimationFrame);
      centerOfMassCalculator.packCenterOfMass(centerOfMassBody);
      FrameVector crossPart = new FrameVector(estimationFrame);
      crossPart.cross(angularVelocityOfEstimationLink, centerOfMassBody);

      // v_{r/p}= \tilde{\omega} r^{estimation} + \dot{r}^{estimation}
      FrameVector centerOfMassVelocityOffset = new FrameVector(estimationFrame);
      centerOfMassVelocityOffset.add(crossPart, comVelocityBody);

      // v_{estimation}^{p,w} = R_{w}^{estimation} \dot{r} - v_{r/p}
      FrameVector estimationLinkVelocity = new FrameVector(estimationFrame);
      estimationLinkVelocity.setAndChangeFrame(centerOfMassVelocityWorld);
      estimationLinkVelocity.changeFrame(estimationFrame);
      estimationLinkVelocity.sub(centerOfMassVelocityOffset);

      return estimationLinkVelocity;
   }

   private Twist computeEstimationLinkTwist(FrameVector estimationLinkAngularVelocity, FrameVector estimationLinkLinearVelocity)
   {
      Twist ret = new Twist(estimationLink.getBodyFixedFrame(), rootJoint.getFrameBeforeJoint(), estimationFrame);
      estimationLinkLinearVelocity.changeFrame(ret.getExpressedInFrame());
      ret.setLinearPart(estimationLinkLinearVelocity.getVector());
      estimationLinkAngularVelocity.changeFrame(ret.getExpressedInFrame());
      ret.setAngularPart(estimationLinkAngularVelocity.getVector());
      
      return ret;
   }

   private Twist computeRootJointTwist(Twist estimationLinkTwist)
   {
      // T_{root}^{root, estimation}
      Twist rootToEstimationTwist = new Twist();
      twistCalculator.packRelativeTwist(rootToEstimationTwist, rootJoint.getSuccessor(), estimationLink);
      rootToEstimationTwist.changeFrame(rootJoint.getFrameAfterJoint());
      rootToEstimationTwist.changeBodyFrameNoRelativeTwist(rootJoint.getFrameAfterJoint());

      // T_{estimation}^{root, w}
      estimationLinkTwist.changeFrame(rootJoint.getFrameAfterJoint());

      // T_{root}^{root, w} = T_{estimation}^{root, w} + T_{root}^{root, estimation}
      Twist rootJointTwist = new Twist(estimationLinkTwist);
      rootJointTwist.add(rootToEstimationTwist);

      return rootJointTwist;
   }

   private void updateRootJointConfiguration()
   {
      FramePoint centerOfMassPositionState = centerOfMassPositionPort.getData();
      FrameOrientation orientationState = orientationPort.getData();

      Transform3D estimationLinkToWorld = computeEstimationLinkTransform(centerOfMassPositionState, orientationState);
      Transform3D rootJointToWorld = computeRootJointTransform(estimationLinkToWorld);
      rootJoint.setPositionAndRotation(rootJointToWorld);
      rootJoint.getFrameAfterJoint().update();
   }

   private Transform3D computeEstimationLinkTransform(FramePoint centerOfMassWorld, FrameOrientation estimationLinkOrientation)
   {
      // r^{estimation}
      centerOfMassCalculator.compute();
      FramePoint centerOfMassBody = new FramePoint(estimationFrame);
      centerOfMassCalculator.packCenterOfMass(centerOfMassBody);
      centerOfMassBody.changeFrame(estimationFrame);

      // R_{estimation}^{w}
      estimationLinkOrientation.changeFrame(worldFrame);
      Transform3D estimationLinkToWorld = new Transform3D();
      estimationLinkOrientation.getTransform3D(estimationLinkToWorld);

      // R_{estimation}^{w} * r^{estimation}
      Vector3d centerOfMassBodyVector3d = new Vector3d();
      centerOfMassBody.getVector(centerOfMassBodyVector3d);
      estimationLinkToWorld.transform(centerOfMassBodyVector3d);

      // p_{estimation}^{w} = r^{w} - R_{estimation}^{w} r^{estimation}
      Point3d estimationLinkPosition = new Point3d();
      centerOfMassWorld.getPoint(estimationLinkPosition);
      estimationLinkPosition.sub(centerOfMassBodyVector3d);

      // H_{estimation}^{w}
      Vector3d estimationLinkPositionVector3d = new Vector3d();
      estimationLinkPositionVector3d.set(estimationLinkPosition);
      estimationLinkToWorld.setTranslation(estimationLinkPositionVector3d);

      return estimationLinkToWorld;
   }


   private Transform3D computeRootJointTransform(Transform3D estimationLinkTransform)
   {
      // H_{root}^{estimation}
      Transform3D rootJointFrameToEstimationFrame = new Transform3D();
      rootJoint.getFrameAfterJoint().getTransformToDesiredFrame(rootJointFrameToEstimationFrame, estimationFrame);

      // H_{root}^{w} = H_{estimation}^{w} * H_{root}^{estimation}
      Transform3D rootJointToWorld = new Transform3D(estimationLinkTransform);
      rootJointToWorld.mul(rootJointFrameToEstimationFrame);

      return rootJointToWorld;
   }

}