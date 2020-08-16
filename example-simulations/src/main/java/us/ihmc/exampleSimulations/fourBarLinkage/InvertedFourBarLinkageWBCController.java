package us.ihmc.exampleSimulations.fourBarLinkage;

import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_SHOULDER_JOINT;
import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_WRIST_JOINT;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageIDController.SineGenerator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunction;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Controller demonstrating the usage of the {@link WholeBodyControllerCore} in the presence of a
 * four bar linkage.
 */
public class InvertedFourBarLinkageWBCController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getName());
   private final RigidBodyBasics rootBody;
   private final FourBarKinematicLoopFunction fourBarKinematicLoop;
   private final WholeBodyControllerCore controllerCore;

   private final Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointMap;

   private final SineGenerator shoulderFunctionGenerator;
   private final SineGenerator fourBarFunctionGenerator;
   private final SineGenerator wristFunctionGenerator;
   private final RevoluteJointBasics shoulderJoint;
   private final RevoluteJointBasics masterJoint;
   private final RevoluteJointBasics jointA;
   private final RevoluteJointBasics jointB;
   private final RevoluteJointBasics jointC;
   private final RevoluteJointBasics jointD;
   private final RevoluteJointBasics wristJoint;

   private final YoPDGains gains = new YoPDGains("", registry);

   private final OneDoFJointBasics[] oneDoFJoints;

   public InvertedFourBarLinkageWBCController(InvertedFourBarLinkageRobotDescription robotDescription, Robot robot, double controlDT)
   {
      rootBody = InvertedFourBarLinkageIDController.toInverseDynamicsRobot(robotDescription);
      shoulderJoint = HAS_SHOULDER_JOINT ? findJoint(robotDescription.getShoulderJointName()) : null;
      jointA = findJoint(robotDescription.getJointAName());
      jointB = findJoint(robotDescription.getJointBName());
      jointC = findJoint(robotDescription.getJointCName());
      jointD = findJoint(robotDescription.getJointDName());
      fourBarKinematicLoop = new FourBarKinematicLoopFunction("fourBar", new RevoluteJointBasics[] {jointA, jointB, jointC, jointD}, 0);
      masterJoint = fourBarKinematicLoop.getMasterJoint();
      wristJoint = HAS_WRIST_JOINT ? findJoint(robotDescription.getWristJointName()) : null;

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
      double masterJointMidRange = 0.5 * (masterJoint.getJointLimitUpper() + masterJoint.getJointLimitLower());
      double masterJointMin = EuclidCoreRandomTools.nextDouble(random, masterJoint.getJointLimitLower(), masterJointMidRange);
      double masterJointMax = EuclidCoreRandomTools.nextDouble(random, masterJointMidRange, masterJoint.getJointLimitUpper());
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

      gains.setKp(10.0);
      gains.setZeta(1.0);
      gains.createDerivativeGainUpdater(true);

      oneDoFJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toArray(OneDoFJointBasics[]::new);

      jointMap = jointCorrespondenceList(rootBody, robot);

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         OneDoFJointFeedbackControlCommand jointCommand = new OneDoFJointFeedbackControlCommand();
         jointCommand.setJoint(joint);
         allPossibleCommands.addCommand(jointCommand);
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
      controllerCoreToolbox.addKinematicLoopFunction(fourBarKinematicLoop);
      controllerCoreToolbox.setupForInverseDynamicsSolver(Collections.emptyList());

      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, new FeedbackControllerTemplate(allPossibleCommands), registry);
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
   }

   @Override
   public void doControl()
   {
      readState();
      fourBarKinematicLoop.updateState(true, false);
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
      fourBarCommand.setJoint(masterJoint);
      fourBarCommand.setInverseDynamics(fourBarFunctionGenerator.getPosition(),
                                        fourBarFunctionGenerator.getVelocity(),
                                        fourBarFunctionGenerator.getAcceleration());
      fourBarCommand.setGains(gains);
      fourBarCommand.setWeightForSolver(1.0);
      feedbackControlCommandList.addCommand(fourBarCommand);

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
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
