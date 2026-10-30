package us.ihmc.avatar.scs2;

import java.util.HashSet;
import java.util.Set;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import gnu.trove.iterator.TObjectDoubleIterator;
import gnu.trove.map.TObjectDoubleMap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine;

/**
 * Sets the armature, the reflected rotor inertia, of each robot joint in the MuJoCo model. The MJCF that SCS2 writes only has one
 * armature for every joint, so this writes the per joint values into the compiled model. The model is compiled again when the
 * world changes, for example when RDX adds environment objects, so this checks for a new model every tick and writes them again.
 * <p>
 * Without armature, a light link like the foot under a stiff joint PD can go unstable, because the simulation computes the PD
 * torque at the estimator rate and MuJoCo holds it over the faster physics ticks in between.
 * </p>
 */
public class SCS2MujocoJointArmatureSetter implements Controller
{
   private final MujocoPhysicsEngine physicsEngine;
   private final String robotName;
   private final TObjectDoubleMap<String> jointArmatures;
   private final Set<String> missingJointsReported = new HashSet<>();
   private long modelAddressWithArmature = 0;

   /**
    * @param jointArmatures armature in kg m^2 by joint name, without the robot name prefix MuJoCo joint names have
    */
   public SCS2MujocoJointArmatureSetter(MujocoPhysicsEngine physicsEngine, String robotName, TObjectDoubleMap<String> jointArmatures)
   {
      this.physicsEngine = physicsEngine;
      this.robotName = robotName;
      this.jointArmatures = jointArmatures;
   }

   @Override
   public void doControl()
   {
      mjModel model = physicsEngine.getDynamicsWorld().getModel();
      if (model == null || model.isNull() || model.address() == modelAddressWithArmature)
         return;

      DoublePointer dofArmature = model.dof_armature();
      IntPointer jointDofAddresses = model.jnt_dofadr();
      int jointsSet = 0;
      for (TObjectDoubleIterator<String> iterator = jointArmatures.iterator(); iterator.hasNext(); )
      {
         iterator.advance();
         int jointId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, robotName + "_" + iterator.key());
         if (jointId < 0)
         {
            if (missingJointsReported.add(iterator.key()))
               LogTools.warn("No MuJoCo joint for {}, its armature was not set", iterator.key());
            continue;
         }
         dofArmature.put(jointDofAddresses.get(jointId), iterator.value());
         jointsSet++;
      }

      // Only the first model is reported, the model is compiled again every time the world changes.
      if (modelAddressWithArmature == 0)
         LogTools.info("Set the MuJoCo armature of {} {} joints", jointsSet, robotName);
      modelAddressWithArmature = model.address();
   }
}
