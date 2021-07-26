package us.ihmc.exampleSimulations.fourBarLinkage;

import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_SHOULDER_JOINT;
import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_WRIST_JOINT;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.stream.Stream;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageIDController.SineGenerator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Controller demonstrating the usage of the {@link WholeBodyControllerCore} in the presence of a
 * four bar linkage.
 */
public class InvertedFourBarOneDoFJointWBCController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getName());
   private final RigidBodyBasics rootBody;
   private final WholeBodyControllerCore controllerCore;

   private final Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointMap;

   private final SineGenerator shoulderFunctionGenerator;
   private final SineGenerator fourBarFunctionGenerator;
   private final SineGenerator wristFunctionGenerator;
   private final RevoluteJointBasics shoulderJoint;
   private final InvertedFourBarJoint fourBarJoint;
   private final RevoluteJointBasics wristJoint;

   private final YoPDGains gains = new YoPDGains("", registry);

   private final OneDoFJointBasics[] oneDoFJoints;

   public InvertedFourBarOneDoFJointWBCController(InvertedFourBarLinkageRobotDescription robotDescription, Robot robot, double controlDT)
   {
      rootBody = toInverseDynamicsRobot("elbow",
                                        robotDescription,
                                        new String[] {robotDescription.getJointAName(), robotDescription.getJointBName(), robotDescription.getJointCName(),
                                              robotDescription.getJointDName()});
      shoulderJoint = HAS_SHOULDER_JOINT ? findJoint(robotDescription.getShoulderJointName()) : null;
      fourBarJoint = findFourBarJoint();
      wristJoint = HAS_WRIST_JOINT ? findJoint(robotDescription.getWristJointName()) : null;

      FramePoint3D temp = new FramePoint3D(fourBarJoint.getPredecessor().getBodyFixedFrame());
      temp.setFromReferenceFrame(fourBarJoint.getJointA().getFrameBeforeJoint());
      temp.setFromReferenceFrame(fourBarJoint.getJointB().getFrameBeforeJoint());
      temp.setFromReferenceFrame(fourBarJoint.getJointC().getFrameBeforeJoint());
      temp.setFromReferenceFrame(fourBarJoint.getJointD().getFrameBeforeJoint());

      Random random = new Random(461);

      if (HAS_SHOULDER_JOINT)
      {
         shoulderFunctionGenerator = new SineGenerator("shoulderFunction", robot.getYoTime(), registry);
         double shoulderRange = shoulderJoint.getJointLimitUpper() - shoulderJoint.getJointLimitLower();
         shoulderFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.0, 0.5 * shoulderRange));
         shoulderFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
         shoulderFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         shoulderFunctionGenerator.setOffset(EuclidCoreRandomTools.nextDouble(random, 0.5 * shoulderRange - shoulderFunctionGenerator.getAmplitude()));
      }
      else
      {
         shoulderFunctionGenerator = null;
      }

      fourBarFunctionGenerator = new SineGenerator("fourBarFunction", robot.getYoTime(), registry);
      double masterJointMidRange = 0.5 * (fourBarJoint.getJointLimitUpper() + fourBarJoint.getJointLimitLower());
      double masterJointMin = EuclidCoreRandomTools.nextDouble(random, fourBarJoint.getJointLimitLower(), masterJointMidRange);
      double masterJointMax = EuclidCoreRandomTools.nextDouble(random, masterJointMidRange, fourBarJoint.getJointLimitUpper());
      fourBarFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.5 * (masterJointMax - masterJointMin)));
      fourBarFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
      fourBarFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
      fourBarFunctionGenerator.setOffset(masterJointMidRange);

      if (HAS_WRIST_JOINT)
      {
         wristFunctionGenerator = new SineGenerator("wristFunction", robot.getYoTime(), registry);
         double wristRange = wristJoint.getJointLimitUpper() - wristJoint.getJointLimitLower();
         wristFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.0, 0.5 * wristRange));
         wristFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
         wristFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         wristFunctionGenerator.setOffset(EuclidCoreRandomTools.nextDouble(random, 0.5 * wristRange - wristFunctionGenerator.getAmplitude()));
      }
      else
      {
         wristFunctionGenerator = null;
      }

      gains.setKp(500.0);
      gains.setZeta(1.0);
      gains.createDerivativeGainUpdater(true);

      oneDoFJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toArray(OneDoFJointBasics[]::new);

      jointMap = jointCorrespondenceList(rootBody, robot);
      jointMap.put(fourBarJoint.getJointA(), (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointA().getName()));
      jointMap.put(fourBarJoint.getJointB(), (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointB().getName()));
      jointMap.put(fourBarJoint.getJointC(), (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointC().getName()));
      jointMap.put(fourBarJoint.getJointD(), (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointD().getName()));

      FeedbackControllerTemplate template = new FeedbackControllerTemplate();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         template.enableOneDoFJointFeedbackController(joint);
      }

      double gravityZ = Math.abs(robot.getGravityZ());
      WholeBodyControlCoreToolbox controllerCoreToolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                                                          gravityZ,
                                                                                          null,
                                                                                          oneDoFJoints,
                                                                                          null,
                                                                                          new RobotArmControllerCoreOptimizationSettings(),
                                                                                          null,
                                                                                          registry);
      controllerCoreToolbox.setupForInverseDynamicsSolver(Collections.emptyList());

      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, template, registry);
   }

   private static RigidBodyBasics toInverseDynamicsRobot(String fourBarJointName, RobotDescription robotDescription, String[] fourBarJointNames)
   {
      RigidBodyBasics rootBody = InvertedFourBarLinkageIDController.toInverseDynamicsRobot(robotDescription);
      RevoluteJointBasics[] fourBarJoints = SubtreeStreams.fromChildren(RevoluteJointBasics.class, rootBody)
                                                          .filter(joint -> Stream.of(fourBarJointNames).anyMatch(name -> name.equals(joint.getName())))
                                                          .toArray(RevoluteJointBasics[]::new);
      InvertedFourBarJoint fourBarJoint = new InvertedFourBarJoint(fourBarJointName, fourBarJoints, 0);

      JointDescription jointDDescription = robotDescription.getJointDescription(fourBarJoint.getJointD().getName());
      fourBarJoint.getJointD().getSuccessor().getChildrenJoints().clear();
      LinkDescription successorDescription = jointDDescription.getLink();
      RigidBody successor = InvertedFourBarLinkageIDController.createRigidBody(fourBarJoint, successorDescription);
      if (InvertedFourBarLinkageRobotDescription.HAS_WRIST_JOINT)
      {
         JointDescription nextJointDescription = jointDDescription.getChildrenJoints().get(0);
         InvertedFourBarLinkageIDController.addJointRecursive(nextJointDescription, successor);
      }
      return rootBody;
   }

   private static Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointCorrespondenceList(RigidBodyBasics rootBody, Robot robot)
   {
      Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointPairs = new LinkedHashMap<>();

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).forEach(joint ->
      {
         OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) robot.getJoint(joint.getName());
         if (scsJoint != null)
            jointPairs.put(joint, scsJoint);
      });
      return jointPairs;
   }

   private RevoluteJointBasics findJoint(String jointName)
   {
      return SubtreeStreams.fromChildren(RevoluteJointBasics.class, rootBody).filter(joint -> joint.getName().equals(jointName)).findFirst().get();
   }

   private InvertedFourBarJoint findFourBarJoint()
   {
      return SubtreeStreams.fromChildren(InvertedFourBarJoint.class, rootBody).findFirst().get();
   }

   @Override
   public void initialize()
   {
   }

   public void readState()
   {
      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> entry : jointMap.entrySet())
      {
         if (entry.getValue() == null)
            continue;
         entry.getKey().setQ(entry.getValue().getQ());
         entry.getKey().setQd(entry.getValue().getQD());
      }

      double qA = jointMap.get(fourBarJoint.getJointA()).getQ();
      double qD = jointMap.get(fourBarJoint.getJointD()).getQ();
      fourBarJoint.setQ(qA + qD);
      double qdA = jointMap.get(fourBarJoint.getJointA()).getQD();
      double qdD = jointMap.get(fourBarJoint.getJointD()).getQD();
      fourBarJoint.setQd(qdA + qdD);
   }

   private final YoDouble tau_check1 = new YoDouble("tau_check_elbow", registry);
   private final YoDouble tau_check2 = new YoDouble("tau_check_jointA", registry);

   @Override
   public void doControl()
   {
      readState();
      rootBody.updateFramesRecursively();

      if (HAS_SHOULDER_JOINT)
         shoulderFunctionGenerator.update();
      fourBarFunctionGenerator.update();
      if (HAS_WRIST_JOINT)
         wristFunctionGenerator.update();

      FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

      if (HAS_SHOULDER_JOINT)
      {
         OneDoFJointFeedbackControlCommand shoulderCommand = new OneDoFJointFeedbackControlCommand();
         shoulderCommand.setJoint(shoulderJoint);
         shoulderCommand.setInverseDynamics(shoulderFunctionGenerator.getPosition(),
                                            shoulderFunctionGenerator.getVelocity(),
                                            shoulderFunctionGenerator.getAcceleration());
         shoulderCommand.setGains(gains);
         shoulderCommand.setWeightForSolver(1.0);
         feedbackControlCommandList.addCommand(shoulderCommand);
      }

      OneDoFJointFeedbackControlCommand fourBarCommand = new OneDoFJointFeedbackControlCommand();
      fourBarCommand.setJoint(fourBarJoint);
      fourBarCommand.setInverseDynamics(fourBarFunctionGenerator.getPosition(),
                                        fourBarFunctionGenerator.getVelocity(),
                                        fourBarFunctionGenerator.getAcceleration());
      fourBarCommand.setGains(gains);
      fourBarCommand.setWeightForSolver(1.0);
      feedbackControlCommandList.addCommand(fourBarCommand);

      Wrench wrench = new Wrench(fourBarJoint.getSuccessor().getBodyFixedFrame(), fourBarJoint.getSuccessor().getBodyFixedFrame());
      wrench.getLinearPart()
            .setMatchingFrame(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 9.81 * fourBarJoint.getSuccessor().getInertia().getMass()));
      wrench.changeFrame(fourBarJoint.getFrameAfterJoint());
      wrench.setBodyFrame(fourBarJoint.getFrameAfterJoint());
      tau_check1.set(fourBarJoint.getUnitJointTwist().dot(wrench));
      wrench.changeFrame(fourBarJoint.getMasterJoint().getFrameAfterJoint());
      wrench.setBodyFrame(fourBarJoint.getMasterJoint().getFrameAfterJoint());
      tau_check2.set(fourBarJoint.getMasterJoint().getUnitJointTwist().dot(wrench));

      if (HAS_WRIST_JOINT)
      {
         OneDoFJointFeedbackControlCommand wristCommand = new OneDoFJointFeedbackControlCommand();
         wristCommand.setJoint(wristJoint);
         wristCommand.setInverseDynamics(wristFunctionGenerator.getPosition(), wristFunctionGenerator.getVelocity(), wristFunctionGenerator.getAcceleration());
         wristCommand.setGains(gains);
         wristCommand.setWeightForSolver(1.0);
         feedbackControlCommandList.addCommand(wristCommand);
      }

      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      controllerCoreCommand.addFeedbackControlCommand(feedbackControlCommandList);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      writeOutput();
   }

   public void writeOutput()
   {
      JointDesiredOutputListReadOnly output = controllerCore.getOutputForLowLevelController();

      for (int i = 0; i < output.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly oneDoFJoint = output.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(i);
         if (jointMap.containsKey(oneDoFJoint))
         {
            jointMap.get(oneDoFJoint).setTau(jointDesiredOutput.getDesiredTorque());
         }
         else if (oneDoFJoint == fourBarJoint)
         {
            fourBarJoint.setTau(jointDesiredOutput.getDesiredTorque());
            jointMap.get(fourBarJoint.getMasterJoint()).setTau(fourBarJoint.getMasterJoint().getTau());
         }
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
