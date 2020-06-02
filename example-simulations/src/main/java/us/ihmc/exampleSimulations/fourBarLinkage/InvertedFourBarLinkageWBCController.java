package us.ihmc.exampleSimulations.fourBarLinkage;

import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_SHOULDER_JOINT;
import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_WRIST_JOINT;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoop;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Controller demonstrating the usage of the {@link WholeBodyControllerCore} in the presence of a
 * four bar linkage.
 */
public class InvertedFourBarLinkageWBCController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getName());
   private final RigidBodyBasics rootBody;
   private final FourBarKinematicLoop fourBarKinematicLoop;
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
      rootBody = toInverseDynamicsRobot(robotDescription);
      shoulderJoint = HAS_SHOULDER_JOINT ? findJoint(robotDescription.getShoulderJointName()) : null;
      jointA = findJoint(robotDescription.getJointAName());
      jointB = findJoint(robotDescription.getJointBName());
      jointC = findJoint(robotDescription.getJointCName());
      jointD = findJoint(robotDescription.getJointDName());
      fourBarKinematicLoop = new FourBarKinematicLoop("fourBar", new RevoluteJointBasics[] {jointA, jointB, jointC, jointD}, 0);
      masterJoint = fourBarKinematicLoop.getMasterJoint();
      wristJoint = HAS_WRIST_JOINT ? findJoint(robotDescription.getWristJointName()) : null;

      Random random = new Random(461);

      if (HAS_SHOULDER_JOINT)
      {
         shoulderFunctionGenerator = new SineGenerator("shoulderFunction", robot.getYoTime());
         double shoulderRange = shoulderJoint.getJointLimitUpper() - shoulderJoint.getJointLimitLower();
         shoulderFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.0, 0.5 * shoulderRange));
         shoulderFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
         shoulderFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         shoulderFunctionGenerator.setOffset(EuclidCoreRandomTools.nextDouble(random, 0.5 * shoulderRange - shoulderFunctionGenerator.amplitude.getValue()));
      }
      else
      {
         shoulderFunctionGenerator = null;
      }

      fourBarFunctionGenerator = new SineGenerator("fourBarFunction", robot.getYoTime());
      double masterJointMidRange = 0.5 * (masterJoint.getJointLimitUpper() + masterJoint.getJointLimitLower());
      double masterJointMin = EuclidCoreRandomTools.nextDouble(random, masterJoint.getJointLimitLower(), masterJointMidRange);
      double masterJointMax = EuclidCoreRandomTools.nextDouble(random, masterJointMidRange, masterJoint.getJointLimitUpper());
      fourBarFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.5 * (masterJointMax - masterJointMin)));
      fourBarFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
      fourBarFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
      fourBarFunctionGenerator.setOffset(masterJointMidRange);

      if (HAS_WRIST_JOINT)
      {
         wristFunctionGenerator = new SineGenerator("wristFunction", robot.getYoTime());
         double wristRange = wristJoint.getJointLimitUpper() - wristJoint.getJointLimitLower();
         wristFunctionGenerator.setAmplitude(EuclidCoreRandomTools.nextDouble(random, 0.0, 0.5 * wristRange));
         wristFunctionGenerator.setFrequency(EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0));
         wristFunctionGenerator.setPhase(EuclidCoreRandomTools.nextDouble(random, Math.PI));
         wristFunctionGenerator.setOffset(EuclidCoreRandomTools.nextDouble(random, 0.5 * wristRange - wristFunctionGenerator.amplitude.getValue()));
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

      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, allPossibleCommands, registry);
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
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   static RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      for (JointDescription rootJoint : description.getRootJoints())
         addLoopClosureConstraintRecursive(rootJoint, rootBody);
      return rootBody;
   }

   static void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint;
      String name = jointDescription.getName();
      Vector3D jointOffset = new Vector3D();
      jointDescription.getOffsetFromParentJoint(jointOffset);

      if (jointDescription instanceof PinJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         PinJointDescription pinJointDescription = (PinJointDescription) jointDescription;
         pinJointDescription.getJointAxis(jointAxis);
         RevoluteJoint revoluteJoint = new RevoluteJoint(name, parentBody, jointOffset, jointAxis);
         revoluteJoint.setJointLimits(pinJointDescription.getLowerLimit(), pinJointDescription.getUpperLimit());
         joint = revoluteJoint;
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         SliderJointDescription sliderJointDescription = (SliderJointDescription) jointDescription;
         sliderJointDescription.getJointAxis(jointAxis);
         PrismaticJoint prismaticJoint = new PrismaticJoint(name, parentBody, jointOffset, jointAxis);
         prismaticJoint.setJointLimits(sliderJointDescription.getLowerLimit(), sliderJointDescription.getUpperLimit());
         joint = prismaticJoint;
      }
      else if (jointDescription instanceof FloatingJointDescription)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.getTranslation().set(jointOffset);
         joint = new SixDoFJoint(name, parentBody, transformToParent);
      }
      else
      {
         throw new IllegalStateException("Joint type not handled.");
      }

      LinkDescription linkDescription = jointDescription.getLink();

      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      RigidBody successor = new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }

   static void addLoopClosureConstraintRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint = parentBody.getChildrenJoints().stream().filter(child -> child.getName().equals(jointDescription.getName())).findFirst().get();
      RigidBodyBasics constraintPredecessor = joint.getSuccessor();

      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         String name = constraintDescription.getName();
         Vector3DBasics offsetFromParentJoint = constraintDescription.getOffsetFromParentJoint();
         Vector3DBasics offsetFromLinkParentJoint = constraintDescription.getOffsetFromLinkParentJoint();
         RigidBodyBasics constraintSuccessor = MultiBodySystemTools.getRootBody(parentBody).subtreeStream()
                                                                   .filter(body -> body.getName().equals(constraintDescription.getLink().getName())).findFirst()
                                                                   .get();

         if (constraintDescription instanceof LoopClosurePinConstraintDescription)
         {
            Vector3DBasics axis = ((LoopClosurePinConstraintDescription) constraintDescription).getAxis();
            RevoluteJoint constraintJoint = new RevoluteJoint(name, constraintPredecessor, offsetFromParentJoint, axis);
            constraintJoint.setupLoopClosure(constraintSuccessor, new RigidBodyTransform(new Quaternion(), offsetFromLinkParentJoint));
         }
         else
         {
            LogTools.error("The constraint type {} is not handled, skipping it.", constraintDescription.getClass().getSimpleName());
         }
      }

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addLoopClosureConstraintRecursive(childJoint, constraintPredecessor);
   }

   private class SineGenerator
   {
      private final YoDouble position;
      private final YoDouble velocity;
      private final YoDouble acceleration;

      private final YoDouble amplitude;
      private final YoDouble frequency;
      private final YoDouble phase;
      private final YoDouble offset;

      private final DoubleProvider time;

      public SineGenerator(String name, DoubleProvider timeProvider)
      {
         this.time = timeProvider;
         position = new YoDouble(name + "Position", registry);
         velocity = new YoDouble(name + "Velocity", registry);
         acceleration = new YoDouble(name + "Acceleration", registry);

         amplitude = new YoDouble(name + "Amplitude", registry);
         frequency = new YoDouble(name + "Frequency", registry);
         phase = new YoDouble(name + "Phase", registry);
         offset = new YoDouble(name + "Offset", registry);
      }

      public void setAmplitude(double amplitude)
      {
         this.amplitude.set(amplitude);
      }

      public void setFrequency(double frequency)
      {
         this.frequency.set(frequency);
      }

      public void setPhase(double phase)
      {
         this.phase.set(phase);
      }

      public void setOffset(double offset)
      {
         this.offset.set(offset);
      }

      public void update()
      {
         double omega = 2.0 * Math.PI * frequency.getValue();
         position.set(offset.getValue() + amplitude.getValue() * Math.sin(omega * time.getValue() + phase.getValue()));
         velocity.set(omega * amplitude.getValue() * Math.cos(omega * time.getValue() + phase.getValue()));
         acceleration.set(-omega * omega * amplitude.getValue() * Math.sin(omega * time.getValue() + phase.getValue()));
      }

      public double getPosition()
      {
         return position.getValue();
      }

      public double getVelocity()
      {
         return velocity.getValue();
      }

      public double getAcceleration()
      {
         return acceleration.getValue();
      }
   }
}
