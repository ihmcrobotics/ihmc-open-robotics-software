package us.ihmc.avatar.scs2;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.outputData.SimulationThreadOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Sim-thread variant of {@link SCS2OutputWriter}: applies the low-level joint PD
 * ({@code tau = tau_ff + kp*(q_d - q) + kd*(qd_d - qd)}) on the simulation thread, i.e. every physics tick
 * (~10 kHz), rather than on the estimator/feedback thread (~500 Hz - 1 kHz). The computed effort is passed straight
 * to the simulation for physics integration.
 *
 * <p>Closing the PD at the physics rate gives tighter tracking than the slower estimator-rate path and mirrors the
 * hardware twitter board, which closes its local PD/current loop at ~4 kHz independent of the 1 kHz EtherCAT setpoint
 * update.
 *
 * <p>This writer carries the SAME safety nets as the estimator-thread {@link SCS2OutputWriter}: the position/velocity
 * feedback max-error clamps AND the unstable-velocity damping backoff (when a joint's velocity reverses repeatedly,
 * its damping is scaled down for a short window). The backoff is load-bearing here: the SCS2 sim joints lack the
 * armature / rotor reflected inertia present on hardware (and assumed by the controller's damping gains), so the bare
 * 10 kHz velocity damping at full commanded gains diverges; the backoff keeps it stable. Once the sim models armature
 * inertia, the backoff should rarely engage. The {@code scs2.simPD.kpScale} / {@code scs2.simPD.kdScale} system
 * properties (default 1.0) scale the applied gains for tuning/diagnostics.
 */
public class SimulationPDOutputWriter implements SimulationThreadOutputWriter
{
   // Diagnostic knobs to scale the low-level PD gains applied on the sim thread (default 1.0 = use the commanded
   // stiffness/damping). Used to probe whether the sim-thread PD instability is a gain-magnitude (esp. damping) issue.
   private static final double KP_SCALE = Double.parseDouble(System.getProperty("scs2.simPD.kpScale", "1.0"));
   private static final double KD_SCALE = Double.parseDouble(System.getProperty("scs2.simPD.kdScale", "1.0"));
   // Set -Dscs2.simPD.backoff=false to disable the unstable-velocity damping backoff (e.g. to validate that a retuned
   // set of damping gains is stable on its own and the backoff never needs to engage).
   private static final boolean BACKOFF_ENABLED = Boolean.parseBoolean(System.getProperty("scs2.simPD.backoff", "true"));
   // Sim-only inertia-scaled damping. When enabled, the commanded kd is replaced by kd = 2*zeta*sqrt(kp*M_ii), where
   // M_ii is the joint's apparent inertia in the SCS2 model (mass-matrix diagonal at the startup pose). This gives a
   // consistent damping ratio across joints whose inertia spans ~1500x (hip ~3.5 vs ankle ~0.0024 kg*m^2), instead of
   // the flat controller gains which over-damp the light distal joints and destabilize them. Does NOT touch the shared
   // controller gain set, so hardware gains are unaffected.
   private static final boolean INERTIA_SCALED_KD = Boolean.parseBoolean(System.getProperty("scs2.simPD.inertiaScaledKd", "true"));
   private static final double INERTIA_SCALED_ZETA = Double.parseDouble(System.getProperty("scs2.simPD.zeta", "1.5"));
   // Sim-only inertia-scaled stiffness: kp = M_ii * omegaN^2, giving every joint the same closed-loop natural
   // frequency omegaN (rad/s) regardless of inertia (the "uniform bandwidth" heuristic). With INERTIA_SCALED_KD on as
   // well, kd = 2*zeta*M_ii*omegaN, so the whole robot has a single (omegaN, zeta). The flat controller kp=10 instead
   // implies omegaN from ~1.7 rad/s (hips) to ~64 rad/s (ankles); the default below (~median) keeps overall stiffness.
   private static final boolean INERTIA_SCALED_KP = Boolean.parseBoolean(System.getProperty("scs2.simPD.inertiaScaledKp", "true"));
   private static final double INERTIA_SCALED_OMEGA_N = Double.parseDouble(System.getProperty("scs2.simPD.omegaN", "5.5"));
   // Gravity-load floor for the inertia-scaled stiffness: kp >= |tau_gravity| / deltaTheta, so a joint can hold its
   // static gravity load within deltaTheta radians of deflection. Combined with the bandwidth rule this gives
   // kp = max(M*omegaN^2, |tau_gravity|/deltaTheta) -- this stops the light posture joints (neck, wrist) from sagging
   // when uniform-bandwidth scaling alone would under-stiffen them. deltaTheta default ~3 degrees.
   private static final double INERTIA_SCALED_DELTA_THETA = Double.parseDouble(System.getProperty("scs2.simPD.deltaTheta", "0.05"));
   private static final double GRAVITY_Z = Double.parseDouble(System.getProperty("scs2.simPD.gravityZ", "-9.81"));

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final List<JointController> jointControllers = new ArrayList<>();
   private final Map<String, JointController> jointControllerMap = new HashMap<>();

   // Unstable-velocity damping backoff (ported from SCS2OutputWriter). When a joint's velocity reverses sign more
   // than unstableVelocityNumberThreshold times in a row (beyond unstableVelocityThreshold), its damping kd is scaled
   // down to unstableVelocityLowDampingScale for unstableVelocityLowDampingDuration seconds. This is the same safety
   // net that keeps the 1 kHz estimator-thread PD alive; without it the 10 kHz sim-thread PD diverges because the
   // commanded damping is too high for the SCS2 sim joints (which lack the rotor/armature inertia present on hardware).
   private final YoDouble unstableVelocityThreshold = new YoDouble("unstableVelocityThreshold", registry);
   private final YoInteger unstableVelocityNumberThreshold = new YoInteger("unstableVelocityNumberThreshold", registry);
   private final YoDouble unstableVelocityLowDampingScale = new YoDouble("unstableVelocityLowDampingScale", registry);
   private final YoDouble unstableVelocityLowDampingDuration = new YoDouble("unstableVelocityLowDampingDuration", registry);

   private boolean apparentInertiaApplied = false;

   public SimulationPDOutputWriter(ControllerInput controllerInput,
                                   ControllerOutput controllerOutput)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;

      unstableVelocityThreshold.set(0.45);
      unstableVelocityNumberThreshold.set(10);
      unstableVelocityLowDampingScale.set(0.25);
      unstableVelocityLowDampingDuration.set(0.5);
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      jointControllers.clear();
      apparentInertiaApplied = false;

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly controllerJoint = jointDesiredOutputList.getOneDoFJoint(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(i);

         if (controllerJoint instanceof CrossFourBarJointBasics)
         {
            CrossFourBarJointBasics controllerFourBarJoint = (CrossFourBarJointBasics) controllerJoint;
            if (controllerOutput.getInput().findJoint(controllerFourBarJoint.getName()) != null)
            {
               OneDoFJointStateBasics simJointInput = controllerOutput.getOneDoFJointOutput(controllerJoint);
               OneDoFJointReadOnly simJointOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
               OneDoFJointController jointController = new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry);
               jointControllers.add(jointController);
               jointControllerMap.put(controllerFourBarJoint.getName(), jointController);
            }
            else
            {
               OneDoFJointStateBasics[] simInputs = new OneDoFJointStateBasics[4];
               simInputs[0] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointA());
               simInputs[1] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointB());
               simInputs[2] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointC());
               simInputs[3] = controllerOutput.getOneDoFJointOutput(controllerFourBarJoint.getJointD());
               OneDoFJointReadOnly[] simOutputs = new OneDoFJointReadOnly[4];
               simOutputs[0] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointA().getName());
               simOutputs[1] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointB().getName());
               simOutputs[2] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointC().getName());
               simOutputs[3] = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerFourBarJoint.getJointD().getName());
               CrossFourBarJointController jointController = new CrossFourBarJointController(controllerFourBarJoint,
                                                                                             simOutputs,
                                                                                             simInputs,
                                                                                             jointDesiredOutput,
                                                                                             registry);
               jointControllers.add(jointController);
               jointControllerMap.put(controllerFourBarJoint.getName(), jointController);
            }
         }
         else
         {
            OneDoFJointStateBasics simJointInput = controllerOutput.getOneDoFJointOutput(controllerJoint);
            OneDoFJointReadOnly simJointOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(controllerJoint.getName());
            OneDoFJointController jointController = new OneDoFJointController(simJointOutput, simJointInput, jointDesiredOutput, registry);
            jointControllers.add(jointController);
            jointControllerMap.put(simJointOutput.getName(), jointController);
         }
      }
   }

   @Override
   public void writeBefore(long timestamp)
   {

   }

   @Override
   public void writeAfter()
   {

   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      if (INERTIA_SCALED_KD && !apparentInertiaApplied)
      {
         computeAndApplyApparentInertias();
         apparentInertiaApplied = true;
      }

      for (int i = 0; i < jointControllers.size(); i++)
      {
         jointControllers.get(i).doControl();
      }
   }

   /**
    * Computes the joint-space mass-matrix diagonal (apparent inertia) of the simulated robot at the current pose and
    * hands each joint controller its inertia so it can size its damping. Done once, lazily, on the first tick so the
    * sim has settled the robot frames.
    */
   private void computeAndApplyApparentInertias()
   {
      MultiBodySystemReadOnly system = controllerInput.getInput();

      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(system);
      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();

      // Gravity term G(q) only: no Coriolis/centrifugal, no joint accelerations. Gives the static torque each joint
      // must hold against gravity at the current pose, used for the kp gravity-load floor.
      InverseDynamicsCalculator gravityCalculator = new InverseDynamicsCalculator(system, false, false);
      gravityCalculator.setGravitationalAcceleration(GRAVITY_Z);
      gravityCalculator.compute();

      int column = 0;
      for (JointReadOnly joint : system.getJointsToConsider())
      {
         if (joint.getDegreesOfFreedom() == 1)
         {
            JointController jointController = jointControllerMap.get(joint.getName());
            if (jointController != null)
            {
               jointController.setApparentInertia(massMatrix.get(column, column));
               jointController.setGravityTorque(gravityCalculator.getComputedJointTau(joint).get(0, 0));
            }
         }
         column += joint.getDegreesOfFreedom();
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private interface JointController
   {
      void doControl();

      void setApparentInertia(double inertia);

      void setGravityTorque(double gravityTorque);
   }

   private class OneDoFJointController implements JointController
   {
      private final OneDoFJointReadOnly simOutput;
      private final OneDoFJointStateBasics simInput;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;

      private final YoInteger unstableVelocityCounter;
      private final YoDouble previousVelocity;
      private final YoDouble unstableVelocityStartTime;

      private double apparentInertia = Double.NaN;
      private double gravityTorque = 0.0;

      public OneDoFJointController(OneDoFJointReadOnly simOutput,
                                   OneDoFJointStateBasics simInput,
                                   JointDesiredOutputReadOnly jointDesiredOutput,
                                   YoRegistry registry)
      {
         this.simOutput = simOutput;
         this.simInput = simInput;
         this.jointDesiredOutput = jointDesiredOutput;

         String prefix = simOutput.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);

         unstableVelocityCounter = new YoInteger(prefix + "UnstableVelocityCounter", registry);
         previousVelocity = new YoDouble(prefix + "PreviousVelocity", registry);
         unstableVelocityStartTime = new YoDouble(prefix + "UnstableVelocityStartTime", registry);
      }

      @Override
      public void setApparentInertia(double inertia)
      {
         this.apparentInertia = inertia;
      }

      @Override
      public void setGravityTorque(double gravityTorque)
      {
         this.gravityTorque = gravityTorque;
      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         if (jointDesiredOutput.hasDesiredTorque())
         {
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         }
         else
         {
            yoControllerTau.set(0.0);
         }

         if (jointDesiredOutput.hasDesiredPosition())
         {
            positionError = jointDesiredOutput.getDesiredPosition()- simOutput.getQ();
         }
         else
         {
            positionError = 0.0;
         }

         if (jointDesiredOutput.hasDesiredVelocity())
         {
            velocityError = jointDesiredOutput.getDesiredVelocity() - simOutput.getQd();
         }
         else
         {
            velocityError = 0.0;
         }

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         double kpValue = (jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0) * KP_SCALE;
         double kdValue = (jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0) * KD_SCALE;
         // Sim-only inertia-scaled stiffness: kp = max(M_ii*omegaN^2, |tau_gravity|/deltaTheta) -- the uniform-
         // bandwidth target, floored so the joint can still hold its static gravity load within deltaTheta.
         if (INERTIA_SCALED_KP && apparentInertia > 0.0)
         {
            double kpBandwidth = apparentInertia * INERTIA_SCALED_OMEGA_N * INERTIA_SCALED_OMEGA_N;
            double kpGravityFloor = Math.abs(gravityTorque) / INERTIA_SCALED_DELTA_THETA;
            kpValue = Math.max(kpBandwidth, kpGravityFloor);
         }
         // Sim-only inertia-scaled damping: kd = 2*zeta*sqrt(kp*M_ii), sized to this joint's apparent inertia.
         if (INERTIA_SCALED_KD && kpValue > 0.0 && apparentInertia > 0.0)
            kdValue = 2.0 * INERTIA_SCALED_ZETA * Math.sqrt(kpValue * apparentInertia);
         kp.set(kpValue);
         kd.set(kdValue);

         if (BACKOFF_ENABLED)
         {
            updateUnstableVelocityCounter();
            double time = controllerInput.getTime();
            if (unstableVelocityCounter.getValue() >= unstableVelocityNumberThreshold.getValue())
               unstableVelocityStartTime.set(time);

            if (time - unstableVelocityStartTime.getValue() <= unstableVelocityLowDampingDuration.getValue())
            {
               double alpha = MathTools.clamp(
                     (time - unstableVelocityStartTime.getValue()) / unstableVelocityLowDampingDuration.getValue(),
                     0.0,
                     1.0);
               kd.mul(EuclidCoreTools.interpolate(unstableVelocityLowDampingScale.getValue(), 1.0, alpha));
            }
         }

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         simInput.setEffort(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
         previousVelocity.set(simOutput.getQd());
      }

      private void updateUnstableVelocityCounter()
      {
         boolean unstable = simOutput.getQd() * previousVelocity.getValue() < 0.0;

         if (unstable)
            unstable = !EuclidCoreTools.epsilonEquals(simOutput.getQd(), previousVelocity.getValue(), unstableVelocityThreshold.getValue());

         if (unstable)
            unstableVelocityCounter.set(Math.min(unstableVelocityCounter.getValue() + 1, unstableVelocityNumberThreshold.getValue()));
         else
            unstableVelocityCounter.set(Math.max(unstableVelocityCounter.getValue() - 1, 0));
      }
   }

   private class CrossFourBarJointController implements JointController
   {
      private final CrossFourBarJoint localFourBarJoint;
      private final OneDoFJointReadOnly[] simOutputs;
      private final int[] torqueSourceIndices;
      private final OneDoFJointStateBasics[] simInputs;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoPositionError, yoVelocityError;
      private final YoDouble yoControllerTau, yoPositionTau, yoVelocityTau;

      private final YoInteger unstableVelocityCounter;
      private final YoDouble previousVelocity;
      private final YoDouble unstableVelocityStartTime;

      private double apparentInertia = Double.NaN;
      private double gravityTorque = 0.0;

      public CrossFourBarJointController(CrossFourBarJointBasics controllerFourBarJoint,
                                         OneDoFJointReadOnly[] simOutputs,
                                         OneDoFJointStateBasics[] simInputs,
                                         JointDesiredOutputReadOnly jointDesiredOutput,
                                         YoRegistry registry)
      {
         this.simOutputs = simOutputs;
         this.simInputs = simInputs;
         this.jointDesiredOutput = jointDesiredOutput;
         localFourBarJoint = CrossFourBarJoint.cloneCrossFourBarJoint(controllerFourBarJoint, ReferenceFrameTools.constructARootFrame("dummy"), "");

         if (controllerFourBarJoint.getJointA().isLoopClosure() || controllerFourBarJoint.getJointD().isLoopClosure())
            torqueSourceIndices = new int[] {1, 2};
         else
            torqueSourceIndices = new int[] {0, 3};

         String prefix = controllerFourBarJoint.getName() + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoPositionError = new YoDouble(prefix + "PositionError", registry);
         yoVelocityError = new YoDouble(prefix + "VelocityError", registry);
         yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
         yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
         yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);

         unstableVelocityCounter = new YoInteger(prefix + "UnstableVelocityCounter", registry);
         previousVelocity = new YoDouble(prefix + "PreviousVelocity", registry);
         unstableVelocityStartTime = new YoDouble(prefix + "UnstableVelocityStartTime", registry);
      }

      @Override
      public void setApparentInertia(double inertia)
      {
         this.apparentInertia = inertia;
      }

      @Override
      public void setGravityTorque(double gravityTorque)
      {
         this.gravityTorque = gravityTorque;
      }

      @Override
      public void doControl()
      {
         double positionError;
         double velocityError;

         updateFourBarJoint();

         if (jointDesiredOutput.hasDesiredTorque())
         {
            yoControllerTau.set(jointDesiredOutput.getDesiredTorque());
         }
         else
         {
            yoControllerTau.set(0.0);
         }

         if (jointDesiredOutput.hasDesiredPosition())
         {
            positionError = jointDesiredOutput.getDesiredPosition() - localFourBarJoint.getQ();
         }
         else
         {
            positionError = 0.0;
         }

         if (jointDesiredOutput.hasDesiredVelocity())
         {
            velocityError = jointDesiredOutput.getDesiredVelocity() - localFourBarJoint.getQd();
         }
         else
         {
            velocityError = 0.0;
         }

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
            positionError = MathTools.clamp(positionError, jointDesiredOutput.getPositionFeedbackMaxError());
         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
            velocityError = MathTools.clamp(velocityError, jointDesiredOutput.getVelocityFeedbackMaxError());

         yoPositionError.set(positionError);
         yoVelocityError.set(velocityError);

         double kpValue = (jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0) * KP_SCALE;
         double kdValue = (jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0) * KD_SCALE;
         // Sim-only inertia-scaled stiffness: kp = max(M_ii*omegaN^2, |tau_gravity|/deltaTheta) -- the uniform-
         // bandwidth target, floored so the joint can still hold its static gravity load within deltaTheta.
         if (INERTIA_SCALED_KP && apparentInertia > 0.0)
         {
            double kpBandwidth = apparentInertia * INERTIA_SCALED_OMEGA_N * INERTIA_SCALED_OMEGA_N;
            double kpGravityFloor = Math.abs(gravityTorque) / INERTIA_SCALED_DELTA_THETA;
            kpValue = Math.max(kpBandwidth, kpGravityFloor);
         }
         // Sim-only inertia-scaled damping: kd = 2*zeta*sqrt(kp*M_ii), sized to this joint's apparent inertia.
         if (INERTIA_SCALED_KD && kpValue > 0.0 && apparentInertia > 0.0)
            kdValue = 2.0 * INERTIA_SCALED_ZETA * Math.sqrt(kpValue * apparentInertia);
         kp.set(kpValue);
         kd.set(kdValue);

         if (BACKOFF_ENABLED)
         {
            updateUnstableVelocityCounter();
            double time = controllerInput.getTime();
            if (unstableVelocityCounter.getValue() >= unstableVelocityNumberThreshold.getValue())
               unstableVelocityStartTime.set(time);

            if (time - unstableVelocityStartTime.getValue() <= unstableVelocityLowDampingDuration.getValue())
            {
               double alpha = MathTools.clamp(
                     (time - unstableVelocityStartTime.getValue()) / unstableVelocityLowDampingDuration.getValue(),
                     0.0,
                     1.0);
               kd.mul(EuclidCoreTools.interpolate(unstableVelocityLowDampingScale.getValue(), 1.0, alpha));
            }
         }

         yoPositionTau.set(kp.getValue() * yoPositionError.getValue());
         yoVelocityTau.set(kd.getValue() * yoVelocityError.getValue());
         double tau_actuated = localFourBarJoint.computeActuatedJointTau(yoControllerTau.getValue() + yoPositionTau.getValue() + yoVelocityTau.getValue());
         /*
          * Ideally we just want to set the torque of the actuated joint, but spreading the torque onto the
          * 2-joint chain that goes through the 4-bar w/o relying on the loop closure makes it a little nicer
          * on SCS's soft constraint.
          */

         for (OneDoFJointStateBasics simInput : simInputs)
         {
            if (simInput != null)
               simInput.setEffort(0.0);
         }

         for (int torqueSourceIndex : torqueSourceIndices)
         {
            double tau = 0.5 * tau_actuated / localFourBarJoint.getFourBarFunction().getLoopJacobian().get(torqueSourceIndex);
            simInputs[torqueSourceIndex].setEffort(tau);
         }

         previousVelocity.set(localFourBarJoint.getQd());
      }

      private void updateFourBarJoint()
      {
         localFourBarJoint.setQ(simOutputs[torqueSourceIndices[0]].getQ() + simOutputs[torqueSourceIndices[1]].getQ());
         localFourBarJoint.setQd(simOutputs[torqueSourceIndices[0]].getQd() + simOutputs[torqueSourceIndices[1]].getQd());
         localFourBarJoint.updateFrame();
      }

      private void updateUnstableVelocityCounter()
      {
         boolean unstable = localFourBarJoint.getQd() * previousVelocity.getValue() < 0.0;

         if (unstable)
            unstable = !EuclidCoreTools.epsilonEquals(localFourBarJoint.getQd(), previousVelocity.getValue(), unstableVelocityThreshold.getValue());

         if (unstable)
            unstableVelocityCounter.set(Math.min(unstableVelocityCounter.getValue() + 1, unstableVelocityNumberThreshold.getValue()));
         else
            unstableVelocityCounter.set(Math.max(unstableVelocityCounter.getValue() - 1, 0));
      }
   }
}