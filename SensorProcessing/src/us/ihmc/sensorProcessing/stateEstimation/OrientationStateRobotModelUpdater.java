package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class OrientationStateRobotModelUpdater extends AbstractControlFlowElement implements Runnable
{
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
   private final ControlFlowPort<FrameOrientation> orientationPort;
   private final ControlFlowPort<FrameVector> angularVelocityPort;

   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // Constructor in case of use as a ControlFlowElement
   public OrientationStateRobotModelUpdater(ControlFlowGraph controlFlowGraph,
           ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort,
           ControlFlowOutputPort<FrameOrientation> orientationOutputPort, ControlFlowOutputPort<FrameVector> angularVelocityOutputPort)
   {
      this.orientationPort = createInputPort("orientationInputPort");
      this.angularVelocityPort = createInputPort("angularVelocityInputPort");
      this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");
      this.inverseDynamicsStructureOutputPort = createOutputPort("inverseDynamicsStructureOutputPort");

      controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);
      controlFlowGraph.connectElements(orientationOutputPort, (ControlFlowInputPort<FrameOrientation>) orientationPort);
      controlFlowGraph.connectElements(angularVelocityOutputPort, (ControlFlowInputPort<FrameVector>) angularVelocityPort);

      this.inverseDynamicsStructureInputPort.setData(inverseDynamicsStructureOutputPort.getData());
      this.inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
   }

   // Constructor in case of use as a Runnable
   public OrientationStateRobotModelUpdater(ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort)
   {
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.inverseDynamicsStructureOutputPort = null;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
   }

   public void run()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();

      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      updateRootJointRotation(inverseDynamicsStructure.getRootJoint(), orientationPort.getData(), estimationFrame);

      rootJoint.getFrameAfterJoint().update();

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      updateRootJointTwistAngularPart(twistCalculator, rootJoint, angularVelocityPort.getData());
      rootJoint.getFrameAfterJoint().update();
      twistCalculator.compute();
   }

   public void startComputation()
   {
      run();
      inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
   }

   public void waitUntilComputationIsDone()
   {
   }

   private final FrameVector tempRootJointAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final Twist tempRootJointTwist = new Twist();

   private void updateRootJointTwistAngularPart(TwistCalculator twistCalculator, FloatingInverseDynamicsJoint rootJoint, FrameVector estimationLinkAngularVelocity)
   {
      rootJoint.getJointTwist(tempRootJointTwist);
      computeRootJointAngularVelocity(twistCalculator, tempRootJointAngularVelocity, estimationLinkAngularVelocity);

      tempRootJointTwist.setAngularPart(tempRootJointAngularVelocity.getVector());
      rootJoint.setJointTwist(tempRootJointTwist);
   }

   private final Twist tempRootToEstimationTwist = new Twist();
   private final FrameVector tempRootToEstimationAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector tempEstimationLinkAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void computeRootJointAngularVelocity(TwistCalculator twistCalculator, FrameVector rootJointAngularVelocityToPack,
           FrameVector angularVelocityEstimationLink)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      FloatingInverseDynamicsJoint rootJoint = inverseDynamicsStructure.getRootJoint();
      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();

      tempEstimationLinkAngularVelocity.setIncludingFrame(angularVelocityEstimationLink);

      // T_{root}^{root, estimation}
      twistCalculator.getRelativeTwist(tempRootToEstimationTwist, estimationLink, rootJoint.getSuccessor());
      tempRootToEstimationTwist.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, estimation}
      tempRootToEstimationAngularVelocity.setToZero(rootJoint.getFrameAfterJoint());
      tempRootToEstimationTwist.getAngularPart(tempRootToEstimationAngularVelocity);

      // omega_{estimation}^{root, world}
      tempEstimationLinkAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());

      // omega_{root}^{root, world} = omega_{estimation}^{root, world} + omega_{root}^{root, estimation}
      rootJointAngularVelocityToPack.setToZero(rootJoint.getFrameAfterJoint());
      rootJointAngularVelocityToPack.add(tempEstimationLinkAngularVelocity, tempRootToEstimationAngularVelocity);
   }

   private final FrameOrientation tempOrientationEstimatinLink = new FrameOrientation(ReferenceFrame.getWorldFrame());    // worldframe just for initializing
   private final RigidBodyTransform tempEstimationLinkToWorld = new RigidBodyTransform();
   private final RigidBodyTransform tempRootJointToWorld = new RigidBodyTransform();

   private void updateRootJointRotation(FloatingInverseDynamicsJoint rootJoint, FrameOrientation estimationLinkOrientation, ReferenceFrame estimationFrame)
   {
      tempOrientationEstimatinLink.setIncludingFrame(estimationLinkOrientation);

      computeEstimationLinkToWorldTransform(tempEstimationLinkToWorld, tempOrientationEstimatinLink);
      computeRootJointToWorldTransform(rootJoint, estimationFrame, tempRootJointToWorld, tempEstimationLinkToWorld);
      RotationMatrix rootJointRotation = new RotationMatrix();
      tempRootJointToWorld.getRotation(rootJointRotation);

      rootJoint.setRotation(rootJointRotation);
   }

   private void computeEstimationLinkToWorldTransform(RigidBodyTransform estimationLinkToWorldToPack, FrameOrientation estimationLinkOrientation)
   {
      // R_{estimation}^{w}
      estimationLinkOrientation.changeFrame(worldFrame);
      estimationLinkOrientation.getTransform3D(estimationLinkToWorldToPack);
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

// private final Twist tempRootJointTwistExisting = new Twist();
// private final FrameVector tempRootJointTwistExistingLinearPart = new FrameVector();

// private void computeRootJointTwistAngularPart(FloatingInverseDynamicsJoint rootJoint, Twist rootJointTwistToPack, FrameVector rootJointAngularVelocity)
// {
//    rootJointAngularVelocity.checkReferenceFrameMatch(rootJoint.getFrameAfterJoint());
//
//    rootJoint.packJointTwist(tempRootJointTwistExisting);
//    tempRootJointTwistExisting.checkReferenceFramesMatch(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
//    tempRootJointTwistExisting.packLinearPart(tempRootJointTwistExistingLinearPart);
//
//    rootJointTwistToPack.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(),
//                             tempRootJointTwistExistingLinearPart.getVector(), rootJointAngularVelocity.getVector());
// }

   public ControlFlowOutputPort<FullInverseDynamicsStructure> getInverseDynamicsStructureOutputPort()
   {
      return inverseDynamicsStructureOutputPort;
   }

   public void initialize()
   {
      startComputation();
      waitUntilComputationIsDone();
   }

   public void initializeOrientionToActual(FrameOrientation actualOrientation)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      updateRootJointRotation(inverseDynamicsStructure.getRootJoint(), actualOrientation, inverseDynamicsStructure.getEstimationFrame());
   }
}
