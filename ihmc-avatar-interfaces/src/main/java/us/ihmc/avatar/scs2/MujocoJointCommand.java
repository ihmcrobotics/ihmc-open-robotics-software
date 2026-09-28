package us.ihmc.avatar.scs2;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoJointActuation;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine;

/**
 * Routes a joint's low-level command to the MuJoCo actuator that drives it, so MuJoCo evaluates
 * {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on every physics step instead of applying a
 * torque computed once per controller tick.
 *
 * <p>This is the whole MuJoCo side of the joint servo. It hooks into {@link SCS2OutputWriter}
 * rather than replacing it because the command has to be taken at the point the effort would have
 * been written: downstream of the corruptors, the velocity scaling, the feedback-error clamps,
 * {@code InterpolatedSCS2OutputWriter} and the low-level output processor. A writer that replaced
 * SCS2OutputWriter would sit upstream of all of them and silently lose them.
 *
 * <p>{@link #setJointCommand} returns false for joints MuJoCo has no actuator for -- cross-four-bars,
 * anything the MJCF builder cannot drive -- and {@link SCS2OutputWriter} writes their effort as it
 * always has.
 *
 * <p>Called from the estimator thread while the engine reads the staged commands on the physics
 * thread. Nothing native is touched here -- {@link MujocoJointActuation} is a plain Java staging
 * object and the engine copies it into {@code mjData} when it steps -- so this is the same benign
 * cross-thread hand-off SCS2 already does through {@code ControllerOutput}.
 */
public class MujocoJointCommand implements JointCommand
{
   private final MujocoPhysicsEngine physicsEngine;
   /** Caches the lookup, including the misses, so an undrivable joint is not re-resolved every tick. */
   private final Map<String, MujocoJointActuation> actuationByJointName = new HashMap<>();
   private boolean warnedAboutUndrivableJoints = false;

   public MujocoJointCommand(MujocoPhysicsEngine physicsEngine)
   {
      this.physicsEngine = physicsEngine;
   }

   @Override
   public boolean setJointCommand(String jointName,
                                  double feedforwardTorque,
                                  double desiredPosition,
                                  double desiredVelocity,
                                  double stiffness,
                                  double damping)
   {
      MujocoJointActuation actuation;
      if (actuationByJointName.containsKey(jointName))
      {
         actuation = actuationByJointName.get(jointName);
      }
      else
      {
         // Not resolvable until the model has compiled, which happens on the engine's first tick.
         actuation = physicsEngine.getJointActuation(jointName);
         if (actuation != null)
            actuationByJointName.put(jointName, actuation);
      }

      if (actuation == null)
      {
         if (!warnedAboutUndrivableJoints)
         {
            warnedAboutUndrivableJoints = true;
            LogTools.warn("No MuJoCo actuator for joint '{}' (and possibly others); those joints keep taking a torque from SCS2OutputWriter.",
                          jointName);
         }
         return false;
      }

      actuation.setCommand(feedforwardTorque, desiredPosition, desiredVelocity, stiffness, damping);
      return true;
   }
}
