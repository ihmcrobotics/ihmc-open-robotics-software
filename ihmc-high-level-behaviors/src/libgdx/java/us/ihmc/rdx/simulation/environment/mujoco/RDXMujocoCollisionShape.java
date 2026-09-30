package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.euclid.geometry.Pose3D;

/**
 * One RDX environment-object collision, expressed in the world frame, ready to drop into the
 * MuJoCo terrain as a mesh, box, or sphere geom.
 */
public final class RDXMujocoCollisionShape
{
   public enum Type
   {
      BOX,
      SPHERE,
      MESH
   }

   private final String name;
   private final Type type;
   private final double sizeX;
   private final double sizeY;
   private final double sizeZ;
   private final String meshResourcePath;
   private final Pose3D poseInWorld;

   public RDXMujocoCollisionShape(String name, Type type, double sizeX, double sizeY, double sizeZ, Pose3D poseInWorld)
   {
      this(name, type, sizeX, sizeY, sizeZ, null, poseInWorld);
   }

   public RDXMujocoCollisionShape(String name, String meshResourcePath, Pose3D poseInWorld)
   {
      this(name, Type.MESH, 0.0, 0.0, 0.0, meshResourcePath, poseInWorld);
   }

   private RDXMujocoCollisionShape(String name,
                                       Type type,
                                       double sizeX,
                                       double sizeY,
                                       double sizeZ,
                                       String meshResourcePath,
                                       Pose3D poseInWorld)
   {
      this.name = name;
      this.type = type;
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
      this.meshResourcePath = meshResourcePath;
      this.poseInWorld = new Pose3D(poseInWorld);
   }

   public String getName()
   {
      return name;
   }

   public Type getType()
   {
      return type;
   }

   public double getSizeX()
   {
      return sizeX;
   }

   public double getSizeY()
   {
      return sizeY;
   }

   public double getSizeZ()
   {
      return sizeZ;
   }

   /** Classpath path of the convex STL, or null for a primitive. */
   public String getMeshResourcePath()
   {
      return meshResourcePath;
   }

   /** File name copied next to world.xml. Unique per geom so two objects can share a hull. */
   public String getMeshFileName()
   {
      return name + ".stl";
   }

   public Pose3D getPoseInWorld()
   {
      return poseInWorld;
   }

   /** Identity of the geom, ignoring the pose, so a drag does not force a MuJoCo recompile. */
   public String structureKey()
   {
      if (type == Type.MESH)
         return name + "|" + type + "|" + meshResourcePath;
      return name + "|" + type + "|" + sizeX + "|" + sizeY + "|" + sizeZ;
   }

   public boolean matches(RDXMujocoCollisionShape other, double epsilon)
   {
      if (other == null || type != other.type || !name.equals(other.name))
         return false;
      if (type == Type.MESH)
      {
         if (meshResourcePath == null ? other.meshResourcePath != null : !meshResourcePath.equals(other.meshResourcePath))
            return false;
      }
      else if (Math.abs(sizeX - other.sizeX) > epsilon || Math.abs(sizeY - other.sizeY) > epsilon || Math.abs(sizeZ - other.sizeZ) > epsilon)
      {
         return false;
      }
      return poseInWorld.getPosition().distance(other.poseInWorld.getPosition()) <= epsilon
             && poseInWorld.getOrientation().distance(other.poseInWorld.getOrientation()) <= epsilon;
   }
}
