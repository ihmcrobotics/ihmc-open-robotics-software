package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Flat ground plus the collisions of RDX environment-builder objects.
 * <p>
 * The object list is published from the UI thread and read by the MuJoCo collision visualizer.
 * MuJoCo itself is updated separately, because its model cannot grow geoms without a recompile.
 */
public class RDXMujocoEnvironment implements CommonAvatarEnvironmentInterface
{
   private final FlatGroundEnvironment ground = new FlatGroundEnvironment();
   private final AtomicReference<List<RDXMujocoCollisionShape>> objectCollisions = new AtomicReference<>(List.of());
   private final AtomicLong revision = new AtomicLong();

   public void setObjectCollisions(List<RDXMujocoCollisionShape> objectCollisions)
   {
      this.objectCollisions.set(List.copyOf(objectCollisions));
      revision.incrementAndGet();
   }

   public List<RDXMujocoCollisionShape> getObjectCollisions()
   {
      return objectCollisions.get();
   }

   public long getRevision()
   {
      return revision.get();
   }

   /** Collision primitives that belong to the flat ground, before any RDX object is added. */
   public int getGroundCollisionShapeCount()
   {
      return ground.getTerrainObject3D().getTerrainCollisionShapes().size();
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      CombinedTerrainObject3D terrain = new CombinedTerrainObject3D("RDXMujocoEnvironment");
      terrain.addTerrainObject(ground.getTerrainObject3D());
      for (RDXMujocoCollisionShape shape : objectCollisions.get())
      {
         RigidBodyTransform pose = new RigidBodyTransform(shape.getPoseInWorld());
         if (shape.getType() == RDXMujocoCollisionShape.Type.BOX)
         {
            terrain.addRotatableBox(pose, shape.getSizeX(), shape.getSizeY(), shape.getSizeZ(), YoAppearance.LightSkyBlue());
         }
         else if (shape.getType() == RDXMujocoCollisionShape.Type.SPHERE)
         {
            terrain.addSphere(shape.getPoseInWorld().getPosition().getX(),
                              shape.getPoseInWorld().getPosition().getY(),
                              shape.getPoseInWorld().getPosition().getZ(),
                              shape.getSizeX(),
                              YoAppearance.LightSkyBlue());
         }
      }
      return terrain;
   }
}
