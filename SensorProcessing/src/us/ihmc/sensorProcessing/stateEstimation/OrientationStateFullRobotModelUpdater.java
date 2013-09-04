package us.ihmc.sensorProcessing.stateEstimation;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoDataInputStream;
import com.yobotics.simulationconstructionset.YoVariable;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassAccelerationCalculator;
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
public class OrientationStateFullRobotModelUpdater implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector> angularAccelerationPort;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public OrientationStateFullRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
         ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort, ControlFlowOutputPort<FrameOrientation> orientationPort,
         ControlFlowOutputPort<FrameVector> angularVelocityPort, ControlFlowOutputPort<FrameVector> angularAccelerationPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;

   }

   public void run()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      updateRootJointRotation(inverseDynamicsStructure.getRootJoint(), orientationPort.getData(), estimationFrame);
      rootJoint.getFrameAfterJoint().update();

      // update rootJoint angular velocity (not the total twist, forget about linear velocity for now)
      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      updateRootJointTwist(twistCalculator, angularVelocityPort.getData());
   }

   private final FrameVector tempRootJointAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final Twist tempRootJointTwist = new Twist();

   private void updateRootJointTwist(TwistCalculator twistCalculator, FrameVector estimationLinkAngularVelocity)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      computeRootJointAngularVelocity(twistCalculator, tempRootJointAngularVelocity, estimationLinkAngularVelocity);
      computeRootJointTwistAngularVelocity(rootJoint, tempRootJointTwist, tempRootJointAngularVelocity);
      rootJoint.setJointTwist(tempRootJointTwist);
   }

   private final Twist tempRootToEstimationTwist = new Twist();
   private final FrameVector tempRootToEstimationAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempCrossTerm = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempEstimationLinkAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void computeRootJointAngularVelocity(TwistCalculator twistCalculator, FrameVector rootJointAngularVelocityToPack,
         FrameVector angularVelocityEstimationLink)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SixDoFJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();

      tempEstimationLinkAngularVelocity.setAndChangeFrame(angularVelocityEstimationLink);

      // T_{root}^{root, estimation}
      twistCalculator.packRelativeTwist(tempRootToEstimationTwist, estimationLink, rootJoint.getSuccessor());
      tempRootToEstimationTwist.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, estimation}
      tempRootToEstimationAngularVelocity.setToZero(rootJoint.getFrameAfterJoint());
      tempRootToEstimationTwist.packAngularPart(tempRootToEstimationAngularVelocity);

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
   }

   private final Twist tempRootJointTwistExisting = new Twist();
   private final FrameVector tempRootJointTwistExistingLinearPart = new FrameVector();

   private void computeRootJointTwistAngularVelocity(SixDoFJoint rootJoint, Twist rootJointTwistToPack, FrameVector rootJointAngularVelocity)
   {
      rootJointAngularVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());

      rootJoint.packJointTwist(tempRootJointTwistExisting);
      tempRootJointTwistExisting.checkReferenceFramesMatch(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      tempRootJointTwistExisting.packLinearPart(tempRootJointTwistExistingLinearPart);

      rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
            tempRootJointTwistExistingLinearPart.getVector(), rootJointAngularVelocity.getVector());
   }

   private final FrameOrientation tempOrientationState = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final Transform3D tempEstimationLinkToWorld = new Transform3D();
   private final Transform3D tempRootJointToWorld = new Transform3D();

   private void updateRootJointRotation(SixDoFJoint rootJoint, FrameOrientation estimationLinkOrientation, ReferenceFrame estimationFrame)
   {
      tempOrientationState.setAndChangeFrame(estimationLinkOrientation);

      computeEstimationLinkToWorldTransform(tempEstimationLinkToWorld, tempOrientationState);
      computeRootJointToWorldTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      Matrix3d rootJointRotation = new Matrix3d();
      tempRootJointToWorld.get(rootJointRotation);
      rootJoint.setRotation(rootJointRotation);
   }

   private void computeEstimationLinkToWorldTransform(Transform3D estimationLinkToWorldToPack, FrameOrientation estimationLinkOrientation)
   {
      // R_{estimation}^{w}
      estimationLinkOrientation.changeFrame(worldFrame);
      estimationLinkOrientation.getTransform3D(estimationLinkToWorldToPack);
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
