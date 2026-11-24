package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.List;

import org.bytedeco.javacpp.BytePointer;

import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * The part of a MuJoCo state that belongs to the environment rather than to a robot: position and velocity of every joint no robot
 * owns, and the mocap poses. That covers free environment objects, articulated doors and other mechanisms, and scenery. Robot
 * joints are the ones named after their robot, as in {@code Alex_LEFT_HIP_X}.
 * <p>
 * Use it to reset only the robots: save the environment, call {@code mj_resetData} and reseed the robots, then restore the
 * environment, so objects stay where they were and keep moving as they were.
 * </p>
 */
public class SCS2MujocoEnvironmentState
{
   private final List<int[]> addresses = new ArrayList<>();
   private final List<double[][]> values = new ArrayList<>();
   private final double[] mocapPositions;
   private final double[] mocapOrientations;

   private SCS2MujocoEnvironmentState(double[] mocapPositions, double[] mocapOrientations)
   {
      this.mocapPositions = mocapPositions;
      this.mocapOrientations = mocapOrientations;
   }

   /**
    * @param robots the robots in the world. Joints whose names start with a robot's name and {@code _} are left out.
    */
   public static SCS2MujocoEnvironmentState save(mjModel model, mjData data, List<? extends Robot> robots)
   {
      int mocapCount = Math.toIntExact(model.nmocap());
      double[] mocapPositions = new double[3 * mocapCount];
      double[] mocapOrientations = new double[4 * mocapCount];
      data.mocap_pos().get(mocapPositions);
      data.mocap_quat().get(mocapOrientations);
      SCS2MujocoEnvironmentState state = new SCS2MujocoEnvironmentState(mocapPositions, mocapOrientations);

      List<String> robotPrefixes = new ArrayList<>();
      for (Robot robot : robots)
         robotPrefixes.add(robot.getName() + "_");
      int jointCount = Math.toIntExact(model.njnt());
      for (int joint = 0; joint < jointCount; joint++)
      {
         String name = jointName(model, joint);
         if (name != null && robotPrefixes.stream().anyMatch(name::startsWith))
            continue;
         int type = model.jnt_type().get(joint);
         int nq = type == Mujoco.mjJNT_FREE ? 7 : type == Mujoco.mjJNT_BALL ? 4 : 1;
         int nv = type == Mujoco.mjJNT_FREE ? 6 : type == Mujoco.mjJNT_BALL ? 3 : 1;
         int qposAddress = model.jnt_qposadr().get(joint);
         int dofAddress = model.jnt_dofadr().get(joint);
         double[] qpos = new double[nq];
         double[] qvel = new double[nv];
         for (int i = 0; i < nq; i++)
            qpos[i] = data.qpos().get(qposAddress + i);
         for (int i = 0; i < nv; i++)
            qvel[i] = data.qvel().get(dofAddress + i);
         state.addresses.add(new int[] {qposAddress, dofAddress});
         state.values.add(new double[][] {qpos, qvel});
      }
      return state;
   }

   /** Writes the saved environment back into {@code data}, which must belong to the same compiled model. */
   public void restore(mjData data)
   {
      data.mocap_pos().put(mocapPositions);
      data.mocap_quat().put(mocapOrientations);
      for (int joint = 0; joint < addresses.size(); joint++)
      {
         int[] address = addresses.get(joint);
         double[][] value = values.get(joint);
         for (int i = 0; i < value[0].length; i++)
            data.qpos().put(address[0] + i, value[0][i]);
         for (int i = 0; i < value[1].length; i++)
            data.qvel().put(address[1] + i, value[1][i]);
      }
   }

   private static String jointName(mjModel model, int joint)
   {
      BytePointer pointer = Mujoco.mj_id2name(model, Mujoco.mjOBJ_JOINT, joint);
      if (pointer == null || pointer.isNull())
         return null;
      String name = pointer.getString();
      int nul = name.indexOf('\0');
      return nul < 0 ? name : name.substring(0, nul);
   }
}
