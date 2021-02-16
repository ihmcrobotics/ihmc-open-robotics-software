package us.ihmc.exampleSimulations.fourBarLinkage;

import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_SHOULDER_JOINT;
import static us.ihmc.exampleSimulations.fourBarLinkage.InvertedFourBarLinkageRobotDescription.HAS_WRIST_JOINT;

import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunction;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Controller demonstrating the usage of the {@link InverseDynamicsCalculator} in the presence of a
 * four bar linkage.
 */
public class InvertedFourBarLinkageIDController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean APPLY_EXTERNAL_FORCE = true;
   private static final Vector3D externalForcePointOffset = new Vector3D(0.05, 0.0, 0.05);
   private static final SpatialVector externalSpatialForce = new SpatialVector(worldFrame, new Vector3D(0, 10, 0), new Vector3D(100, 0, 0));

   private final YoRegistry registry = new YoRegistry(getName());
   private final RigidBodyBasics rootBody;
   private final FourBarKinematicLoopFunction fourBarKinematicLoop;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final PairList<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointPairs;

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
   private final MultiBodySystemBasics multiBodySystem;

   private final YoDouble kp = new YoDouble("kp", registry);
   private final YoDouble zeta = new YoDouble("zeta", registry);

   private final OneDoFJointBasics[] oneDoFJoints;
   private final YoDouble[] controllerJointPositions;
   private final YoDouble[] controllerJointVelocities;
   private final YoDouble[] desiredJointAccelerations;
   private final YoDouble[] inverseDynamicsJointEfforts;

   private ExternalForcePoint wristExternalForcePoint = null;

   public InvertedFourBarLinkageIDController(InvertedFourBarLinkageRobotDescription robotDescription, Robot robot)
   {
      rootBody = toInverseDynamicsRobot(robotDescription);
      multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(rootBody);
      shoulderJoint = HAS_SHOULDER_JOINT ? findJoint(robotDescription.getShoulderJointName()) : null;
      jointA = findJoint(robotDescription.getJointAName());
      jointB = findJoint(robotDescription.getJointBName());
      jointC = findJoint(robotDescription.getJointCName());
      jointD = findJoint(robotDescription.getJointDName());
      fourBarKinematicLoop = new FourBarKinematicLoopFunction("fourBar", new RevoluteJointBasics[] {jointA, jointB, jointC, jointD}, 0);
      masterJoint = fourBarKinematicLoop.getMasterJoint();
      wristJoint = HAS_WRIST_JOINT ? findJoint(robotDescription.getWristJointName()) : null;

      inverseDynamicsCalculator = new InverseDynamicsCalculator(rootBody);
      inverseDynamicsCalculator.setGravitionalAcceleration(robot.getGravityX(), robot.getGravityY(), robot.getGravityZ());

      Random random = new Random(461);

      if (HAS_SHOULDER_JOINT)
      {
         shoulderFunctionGenerator = new SineGenerator("shoulderFunction", robot.getYoTime(), registry);
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
         wristFunctionGenerator.setOffset(EuclidCoreRandomTools.nextDouble(random, 0.5 * wristRange - wristFunctionGenerator.amplitude.getValue()));

         if (APPLY_EXTERNAL_FORCE)
         {
            wristExternalForcePoint = new ExternalForcePoint("disturbancePoint", externalForcePointOffset, robot);
            wristExternalForcePoint.getYoForce().set(externalSpatialForce.getLinearPart());
            wristExternalForcePoint.getYoMoment().set(externalSpatialForce.getAngularPart());
            robot.getJoint(robotDescription.getWristJointName()).addExternalForcePoint(wristExternalForcePoint);
         }
      }
      else
      {
         wristFunctionGenerator = null;
      }

      kp.set(10.0);
      zeta.set(1.0);

      oneDoFJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toArray(OneDoFJointBasics[]::new);
      controllerJointPositions = new YoDouble[oneDoFJoints.length];
      controllerJointVelocities = new YoDouble[oneDoFJoints.length];
      desiredJointAccelerations = new YoDouble[oneDoFJoints.length];
      inverseDynamicsJointEfforts = new YoDouble[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         controllerJointPositions[i] = new YoDouble("q_ctrl_" + oneDoFJoints[i].getName(), registry);
         controllerJointVelocities[i] = new YoDouble("qd_ctrl_" + oneDoFJoints[i].getName(), registry);
         desiredJointAccelerations[i] = new YoDouble("qdd_d_" + oneDoFJoints[i].getName(), registry);
         inverseDynamicsJointEfforts[i] = new YoDouble("tau_id_" + oneDoFJoints[i].getName(), registry);
      }

      jointPairs = jointCorrespondenceList(rootBody, robot);
   }

   private static PairList<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointCorrespondenceList(RigidBodyBasics rootBody, Robot robot)
   {
      PairList<OneDoFJointBasics, OneDegreeOfFreedomJoint> jointPairs = new PairList<>();

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).forEach(joint ->
      {
         OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) robot.getJoint(joint.getName());
         jointPairs.add(joint, scsJoint);
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
      for (int i = 0; i < jointPairs.size(); i++)
      {
         ImmutablePair<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair = jointPairs.get(i);
         if (pair.getRight() == null)
            continue;
         pair.getLeft().setQ(pair.getRight().getQ());
         pair.getLeft().setQd(pair.getRight().getQD());
      }
   }

   @Override
   public void doControl()
   {
      readState();
      fourBarKinematicLoop.updateState(true, false);
      rootBody.updateFramesRecursively();
      inverseDynamicsCalculator.setExternalWrenchesToZero();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         controllerJointPositions[i].set(oneDoFJoints[i].getQ());
         controllerJointVelocities[i].set(oneDoFJoints[i].getQd());
      }

      if (HAS_SHOULDER_JOINT)
         shoulderFunctionGenerator.update();
      fourBarFunctionGenerator.update();
      if (HAS_WRIST_JOINT)
         wristFunctionGenerator.update();

      double kd = GainCalculator.computeDerivativeGain(kp.getValue(), zeta.getValue());

      if (HAS_SHOULDER_JOINT)
      {
         double q_d_shoulder = shoulderFunctionGenerator.getPosition();
         double q_shoulder = shoulderJoint.getQ();
         double qd_d_shoulder = shoulderFunctionGenerator.getVelocity();
         double qd_shoulder = shoulderJoint.getQd();
         double qdd_d_shoulder = kp.getValue() * (q_d_shoulder - q_shoulder) + kd * (qd_d_shoulder - qd_shoulder) + shoulderFunctionGenerator.getAcceleration();
         shoulderJoint.setQdd(qdd_d_shoulder);
      }

      double q_d_master = fourBarFunctionGenerator.getPosition();
      double q_master = masterJoint.getQ();
      double qd_d_master = fourBarFunctionGenerator.getVelocity();
      double qd_master = masterJoint.getQd();
      double qdd_d_master = kp.getValue() * (q_d_master - q_master) + kd * (qd_d_master - qd_master) + fourBarFunctionGenerator.getAcceleration();
      masterJoint.setQdd(qdd_d_master);

      if (HAS_WRIST_JOINT)
      {
         double q_d_wrist = wristFunctionGenerator.getPosition();
         double q_wrist = wristJoint.getQ();
         double qd_d_wrist = wristFunctionGenerator.getVelocity();
         double qd_wrist = wristJoint.getQd();
         double qdd_d_wrist = kp.getValue() * (q_d_wrist - q_wrist) + kd * (qd_d_wrist - qd_wrist) + wristFunctionGenerator.getAcceleration();
         wristJoint.setQdd(qdd_d_wrist);

         if (APPLY_EXTERNAL_FORCE)
         {
            RigidBodyBasics hand = wristJoint.getSuccessor();
            MovingReferenceFrame jointFrame = wristJoint.getFrameAfterJoint();
            SpatialVector spatialVector = new SpatialVector(externalSpatialForce);
            spatialVector.changeFrame(jointFrame);
            Wrench externalWrench = new Wrench(hand.getBodyFixedFrame(), jointFrame);
            externalWrench.set(spatialVector.getAngularPart(), spatialVector.getLinearPart(), new FramePoint3D(jointFrame, externalForcePointOffset));
            externalWrench.changeFrame(hand.getBodyFixedFrame());
            inverseDynamicsCalculator.setExternalWrench(hand, externalWrench);
         }
      }

      fourBarKinematicLoop.updateState(true, true);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         desiredJointAccelerations[i].set(oneDoFJoints[i].getQdd());
      }

      inverseDynamicsCalculator.compute();
      inverseDynamicsCalculator.writeComputedJointWrenches(multiBodySystem.getAllJoints());
      fourBarKinematicLoop.updateEffort();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         inverseDynamicsJointEfforts[i].set(oneDoFJoints[i].getTau());
      }

      writeOutput();
   }

   public void writeOutput()
   {
      for (int i = 0; i < jointPairs.size(); i++)
      {
         ImmutablePair<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair = jointPairs.get(i);
         if (pair.getRight() == null)
            continue;
         pair.getRight().setTau(pair.getLeft().getTau());
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   static RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", worldFrame);
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      for (JointDescription rootJoint : description.getRootJoints())
         addLoopClosureConstraintRecursive(rootJoint, rootBody);
      return rootBody;
   }

   static void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint = createJoint(jointDescription, parentBody);
      RigidBody successor = createRigidBody(joint, jointDescription.getLink());

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }

   static JointBasics createJoint(JointDescription jointDescription, RigidBodyBasics parentBody)
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
      return joint;
   }

   static RigidBody createRigidBody(JointBasics joint, LinkDescription linkDescription)
   {
      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      return new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);
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

   static class SineGenerator
   {
      private final YoDouble position;
      private final YoDouble velocity;
      private final YoDouble acceleration;

      private final YoDouble amplitude;
      private final YoDouble frequency;
      private final YoDouble phase;
      private final YoDouble offset;

      private final DoubleProvider time;

      public SineGenerator(String name, DoubleProvider timeProvider, YoRegistry registry)
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

      public double getAmplitude()
      {
         return amplitude.getValue();
      }

      public double getFrequency()
      {
         return frequency.getValue();
      }

      public double getPhase()
      {
         return phase.getValue();
      }

      public double getOffset()
      {
         return offset.getValue();
      }
   }
}
