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
   /** False for the wall, which stays a mocap body. Everything else is a free body the robot can move. */
   private final boolean dynamic;
   private final double mass;
   private final boolean selected;

   public RDXMujocoCollisionShape(String name, Type type, double sizeX, double sizeY, double sizeZ, Pose3D poseInWorld,
                                  boolean dynamic, double mass, boolean selected)
   {
      this(name, type, sizeX, sizeY, sizeZ, null, poseInWorld, dynamic, mass, selected);
   }

   public RDXMujocoCollisionShape(String name, String meshResourcePath, Pose3D poseInWorld,
                                  boolean dynamic, double mass, boolean selected)
   {
      this(name, Type.MESH, 0.0, 0.0, 0.0, meshResourcePath, poseInWorld, dynamic, mass, selected);
   }

   private RDXMujocoCollisionShape(String name,
                                       Type type,
                                       double sizeX,
                                       double sizeY,
                                       double sizeZ,
                                       String meshResourcePath,
                                       Pose3D poseInWorld,
                                       boolean dynamic,
                                       double mass,
                                       boolean selected)
   {
      this.name = name;
      this.type = type;
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
      this.meshResourcePath = meshResourcePath;
      this.poseInWorld = new Pose3D(poseInWorld);
      this.dynamic = dynamic;
      this.mass = mass;
      this.selected = selected;
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

   /**
    * Body that carries this geom. MuJoCo's broadphase uses a compile-time bounding box for geoms
    * written straight into the worldbody, so a geom moved at runtime is never tested for contact.
    * Each collision therefore gets its own body, whose bounding box follows its pose.
    */
   public String getBodyName()
   {
      return name + "_body";
   }

   /** Free joint of a dynamic object. The wall has none. */
   public String getJointName()
   {
      return name + "_free";
   }

   public boolean isDynamic()
   {
      return dynamic;
   }

   public double getMass()
   {
      return mass;
   }

   /** True while the user is dragging this object, so physics follows the gizmo instead of integrating. */
   public boolean isSelected()
   {
      return selected;
   }

   public Pose3D getPoseInWorld()
   {
      return poseInWorld;
   }

   /** Identity of the geom, ignoring the pose, so a drag does not force a MuJoCo recompile. */
   public String structureKey()
   {
      String motion = (dynamic ? "free" : "mocap") + "|" + mass;
      if (type == Type.MESH)
         return name + "|" + type + "|" + meshResourcePath + "|" + motion;
      return name + "|" + type + "|" + sizeX + "|" + sizeY + "|" + sizeZ + "|" + motion;
   }

   public boolean matches(RDXMujocoCollisionShape other, double epsilon)
   {
      if (other == null || type != other.type || !name.equals(other.name) || dynamic != other.dynamic || selected != other.selected)
         return false;
      if (Math.abs(mass - other.mass) > epsilon)
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
