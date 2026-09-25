package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoJointActuation;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoActuationMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.SimulationThreadOutputWriter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Forwards the controller's low-level command to MuJoCo instead of collapsing it into a torque.
 *
 * <p>{@link SCS2OutputWriter} computes {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on the
 * estimator thread and writes the result as a joint effort, which the engine then holds constant
 * across every physics step until the next controller tick. The hardware does not work that way:
 * the drives are sent the setpoints and gains and close the loop themselves at their own rate. This
 * writer sends MuJoCo the same five numbers, and {@link MujocoActuationMode#JOINT_SERVO} has MuJoCo
 * evaluate the law on every physics step.
 *
 * <p>The torque decomposition stays visible. It is published by the engine, from the per-actuator
 * forces MuJoCo reports, under the {@code <joint>LowLevel{Controller,Position,Velocity}Tau} names
 * this pipeline has always used -- now the forces actually applied rather than a controller-rate
 * prediction of them. This writer publishes the command that produced them.
 *
 * <p>It runs on the simulation thread rather than the estimator thread because it writes into
 * native {@code mjModel} / {@code mjData}, which is not safe to do while {@code mj_step} is
 * running on another thread. The simulation-thread hook runs inside the engine's controller update,
 * immediately before the step.
 *
 * <p>Joints MuJoCo has no actuator for -- cross-four-bars, welded subtrees, anything the MJCF
 * builder cannot map -- fall back to writing the effort, exactly as before.
 */
public class MujocoSetpointOutputWriter implements SimulationThreadOutputWriter
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ControllerInput controllerInput;
   private final ControllerOutput controllerOutput;
   private final MujocoPhysicsEngine physicsEngine;

   private final List<JointCommandForwarder> jointForwarders = new ArrayList<>();

   public MujocoSetpointOutputWriter(ControllerInput controllerInput, ControllerOutput controllerOutput, MujocoPhysicsEngine physicsEngine)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.physicsEngine = physicsEngine;

      if (physicsEngine.getActuationMode() != MujocoActuationMode.JOINT_SERVO)
      {
         throw new IllegalArgumentException("MujocoSetpointOutputWriter needs the engine in " + MujocoActuationMode.JOINT_SERVO
                                            + ", but it is in " + physicsEngine.getActuationMode()
                                            + ". Set the actuation mode on the MujocoSimulationParameters before the simulation is created.");
      }
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      jointForwarders.clear();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly controllerJoint = jointDesiredOutputList.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(i);

         String jointName = controllerJoint.getName();
         OneDoFJointReadOnly simOutput = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(jointName);
         if (simOutput == null)
            continue;
         OneDoFJointStateBasics simInput = controllerOutput.getOneDoFJointOutput(jointName);

         jointForwarders.add(new JointCommandForwarder(jointName, simOutput, simInput, jointDesiredOutput, registry));
      }
   }

   @Override
   public void initialize()
   {
      // The actuation blocks only exist once MuJoCo has compiled the model, which happens on the
      // engine's first initialize(). Bind them here rather than in the constructor.
      int unmappedCount = 0;
      for (int i = 0; i < jointForwarders.size(); i++)
      {
         if (!jointForwarders.get(i).bind(physicsEngine))
            unmappedCount++;
      }
      if (unmappedCount > 0)
      {
         LogTools.warn("{} of {} joints have no MuJoCo actuator and will fall back to writing effort directly. "
                       + "Cross-four-bar joints and ignored subtrees are expected here.", unmappedCount, jointForwarders.size());
      }
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < jointForwarders.size(); i++)
         jointForwarders.get(i).doControl();
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
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private static class JointCommandForwarder
   {
      private final String jointName;
      private final OneDoFJointReadOnly simOutput;
      private final OneDoFJointStateBasics simInput;
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      private final YoDouble kp, kd;
      private final YoDouble yoDesiredPosition, yoDesiredVelocity, yoFeedforwardTau;

      /** Null when the joint has no MuJoCo actuator, in which case the effort is written instead. */
      private MujocoJointActuation actuation;

      private JointCommandForwarder(String jointName,
                                    OneDoFJointReadOnly simOutput,
                                    OneDoFJointStateBasics simInput,
                                    JointDesiredOutputReadOnly jointDesiredOutput,
                                    YoRegistry registry)
      {
         this.jointName = jointName;
         this.simOutput = simOutput;
         this.simInput = simInput;
         this.jointDesiredOutput = jointDesiredOutput;

         String prefix = jointName + "LowLevel";
         kp = new YoDouble(prefix + "Kp", registry);
         kd = new YoDouble(prefix + "Kd", registry);
         yoDesiredPosition = new YoDouble(prefix + "DesiredPosition", registry);
         yoDesiredVelocity = new YoDouble(prefix + "DesiredVelocity", registry);
         yoFeedforwardTau = new YoDouble(prefix + "FeedforwardTau", registry);
      }

      private boolean bind(MujocoPhysicsEngine physicsEngine)
      {
         actuation = physicsEngine.getJointActuation(jointName);
         return actuation != null;
      }

      private void doControl()
      {
         double feedforwardTorque = jointDesiredOutput.hasDesiredTorque() ? jointDesiredOutput.getDesiredTorque() : 0.0;
         double stiffness = jointDesiredOutput.hasStiffness() ? jointDesiredOutput.getStiffness() : 0.0;
         double damping = jointDesiredOutput.hasDamping() ? jointDesiredOutput.getDamping() : 0.0;

         // A missing setpoint has to become a zero gain rather than a zero setpoint, or the joint
         // would be commanded to zero instead of being left alone on that term.
         double desiredPosition = simOutput.getQ();
         if (jointDesiredOutput.hasDesiredPosition())
         {
            // The max-error limits clamp the error in SCS2OutputWriter. An affine MuJoCo actuator
            // cannot express that, so apply it as a clamped setpoint instead, using the measurement
            // from this controller tick. That makes the limit one tick stale; it is a safety
            // limiter that is rarely active, so this is the intended approximation.
            desiredPosition = jointDesiredOutput.getClampedDesiredPosition(simOutput.getQ());
         }
         else
         {
            stiffness = 0.0;
         }

         double desiredVelocity = simOutput.getQd();
         if (jointDesiredOutput.hasDesiredVelocity())
         {
            desiredVelocity = jointDesiredOutput.getClampedDesiredVelocity(simOutput.getQd());
            if (jointDesiredOutput.hasVelocityScaling())
               desiredVelocity *= jointDesiredOutput.getVelocityScaling();
         }
         else
         {
            damping = 0.0;
         }

         kp.set(stiffness);
         kd.set(damping);
         yoDesiredPosition.set(desiredPosition);
         yoDesiredVelocity.set(desiredVelocity);
         yoFeedforwardTau.set(feedforwardTorque);

         if (actuation != null)
         {
            actuation.setCommand(feedforwardTorque, desiredPosition, desiredVelocity, stiffness, damping);
         }
         else
         {
            // No actuator for this joint: close the loop here, as the other output writers do.
            simInput.setEffort(feedforwardTorque + stiffness * (desiredPosition - simOutput.getQ()) + damping * (desiredVelocity - simOutput.getQd()));
         }
      }
   }
}
