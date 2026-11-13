package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;

import java.util.Map;

/**
 * One articulated object as {@link RDXMujocoObjectCollisionSync} compiles it into the MuJoCo world.
 * A fresh snapshot is taken every UI frame. The binding from {@link #bind(mjModel)} lasts as long as
 * the compiled model, so state that has to survive between frames (a door latch, for example) lives there.
 */
public interface RDXMujocoArticulatedDynamics
{
   /** Prefix of every body, joint, geom and mesh name this object adds. Unique per placed object. */
   String getBaseName();

   /** Changes whenever the compiled structure would change. Poses and joint positions are not part of it. */
   String structureKey();

   /** Degrees of freedom this object adds to the model, so the sync can check the recompiled nq and nv. */
   int getDegreesOfFreedom();

   /** Mesh files to copy beside the world XML: file name there, mapped to the absolute source path. */
   Map<String, String> getMeshFiles();

   void appendAssets(StringBuilder assets);

   void appendBodies(StringBuilder bodies);

   void appendExcludes(StringBuilder contact);

   /** Looks up this object's ids in a freshly compiled model, or returns null when they are missing. */
   Object bind(mjModel model);

   /**
    * Writes the placement and, when seeding or overridden, the commanded joint positions. Returns the
    * joint positions after this update, which go back to {@link RDXMujocoArticulatedObject#applySimulatedJointPositions}.
    */
   double[] syncJoints(mjModel model, mjData data, Object binding, boolean seedFromCommand);
}
