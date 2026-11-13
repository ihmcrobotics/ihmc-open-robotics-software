package us.ihmc.rdx.simulation.environment.isaac;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

/**
 * One rigid link of an articulated environment object, as {@link RDXIsaacSimEnvironmentExporter}
 * writes it into a USD articulation. The link frame is the joint frame at zero joint position.
 */
public final class RDXIsaacSimLink
{
   public enum JointType
   {
      /** The first link. It is welded to the world. */
      ROOT, FIXED, REVOLUTE, CONTINUOUS, PRISMATIC
   }

   /** A visual model and its pose in the link frame. */
   public record Visual(Model model, RigidBodyTransformReadOnly poseInLink)
   {
   }

   /** A convex collision: a mesh whose hull collides, or a box of {@code size} when the model is null. */
   public record Collision(Model model, Vector3DReadOnly size, RigidBodyTransformReadOnly poseInLink)
   {
      public static Collision mesh(Model model, RigidBodyTransformReadOnly poseInLink)
      {
         return new Collision(model, null, new RigidBodyTransform(poseInLink));
      }

      public static Collision box(double sizeX, double sizeY, double sizeZ, RigidBodyTransformReadOnly poseInLink)
      {
         return new Collision(null, new Vector3D(sizeX, sizeY, sizeZ), new RigidBodyTransform(poseInLink));
      }
   }

   private final String name;
   private final int parentIndex;
   private final JointType jointType;
   private final RigidBodyTransform jointFrameInParent = new RigidBodyTransform();
   private final Vector3D axis = new Vector3D(1.0, 0.0, 0.0);
   private double lowerLimit = Double.NEGATIVE_INFINITY;
   private double upperLimit = Double.POSITIVE_INFINITY;
   private double position;
   private String jointName;
   private double damping;
   private double stiffness;
   private double mass;
   private double density;
   private Point3D centerOfMass;
   private final List<Visual> visuals = new ArrayList<>();
   private final List<Collision> collisions = new ArrayList<>();

   private RDXIsaacSimLink(String name, int parentIndex, JointType jointType)
   {
      this.name = name;
      this.parentIndex = parentIndex;
      this.jointType = jointType;
   }

   public static RDXIsaacSimLink root(String name)
   {
      return new RDXIsaacSimLink(name, -1, JointType.ROOT);
   }

   /**
    * @param jointFrameInParent where the joint is in the parent link frame. The child link frame is
    *                           this frame moved by the joint position.
    * @param axis               rotation or slide axis in the joint frame. Ignored for a fixed joint.
    */
   public static RDXIsaacSimLink child(String name, int parentIndex, JointType jointType, RigidBodyTransformReadOnly jointFrameInParent, Vector3DReadOnly axis)
   {
      if (jointType == JointType.ROOT)
         throw new IllegalArgumentException("Only the first link is the root");
      RDXIsaacSimLink link = new RDXIsaacSimLink(name, parentIndex, jointType);
      link.jointFrameInParent.set(jointFrameInParent);
      if (axis != null && axis.norm() > 1.0e-9)
      {
         link.axis.set(axis);
         link.axis.normalize();
      }
      return link;
   }

   /** Name of the joint to the parent. Defaults to {@code <link>_joint}. */
   public RDXIsaacSimLink setJointName(String jointName)
   {
      this.jointName = jointName;
      return this;
   }

   /** Radians, or meters for a prismatic joint. Infinite means unlimited. */
   public RDXIsaacSimLink setLimits(double lowerLimit, double upperLimit)
   {
      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;
      return this;
   }

   /** Current joint position, radians or meters. */
   public RDXIsaacSimLink setPosition(double position)
   {
      this.position = position;
      return this;
   }

   /** Damping in N m s/rad (N s/m) and a spring to zero in N m/rad (N/m), as MuJoCo uses them. */
   public RDXIsaacSimLink setDrive(double damping, double stiffness)
   {
      this.damping = damping;
      this.stiffness = stiffness;
      return this;
   }

   /** Kilograms. The inertia is computed from the collisions. */
   public RDXIsaacSimLink setMass(double mass, Point3DReadOnly centerOfMass)
   {
      this.mass = mass;
      this.centerOfMass = centerOfMass == null ? null : new Point3D(centerOfMass);
      return this;
   }

   /** Kilograms per cubic meter, used when no mass is given. */
   public RDXIsaacSimLink setDensity(double density)
   {
      this.density = density;
      return this;
   }

   public RDXIsaacSimLink addVisual(Model model, RigidBodyTransformReadOnly poseInLink)
   {
      if (model != null)
         visuals.add(new Visual(model, new RigidBodyTransform(poseInLink)));
      return this;
   }

   public RDXIsaacSimLink addCollision(Collision collision)
   {
      collisions.add(collision);
      return this;
   }

   public String getName()
   {
      return name;
   }

   /** Null when the link name should be used. */
   public String getJointName()
   {
      return jointName;
   }

   public int getParentIndex()
   {
      return parentIndex;
   }

   public JointType getJointType()
   {
      return jointType;
   }

   public RigidBodyTransformReadOnly getJointFrameInParent()
   {
      return jointFrameInParent;
   }

   public Vector3DReadOnly getAxis()
   {
      return axis;
   }

   public double getLowerLimit()
   {
      return lowerLimit;
   }

   public double getUpperLimit()
   {
      return upperLimit;
   }

   public double getPosition()
   {
      return position;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getStiffness()
   {
      return stiffness;
   }

   public double getMass()
   {
      return mass;
   }

   public double getDensity()
   {
      return density;
   }

   /** Null when PhysX should compute it from the collisions. */
   public Point3DReadOnly getCenterOfMass()
   {
      return centerOfMass;
   }

   public List<Visual> getVisuals()
   {
      return visuals;
   }

   public List<Collision> getCollisions()
   {
      return collisions;
   }
}
