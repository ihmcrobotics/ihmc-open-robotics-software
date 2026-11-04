package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainSurface;
import us.ihmc.simulationConstructionSetTools.util.ground.ConvexPolytopeTerrainObject;
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
   /** Procedural ground has holes for the terrain below z = 0, so it replaces the flat ground, which would fill them. */
   private volatile boolean replaceFlatGround;

   /** Call before {@link #setObjectCollisions}, so the recompile it triggers sees the new value. */
   public void setReplaceFlatGround(boolean replaceFlatGround)
   {
      this.replaceFlatGround = replaceFlatGround;
   }

   public boolean isFlatGroundReplaced()
   {
      return replaceFlatGround;
   }

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
      if (!replaceFlatGround)
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
         else if (shape.getType() == RDXMujocoCollisionShape.Type.TERRAIN)
         {
            for (TerrainBox box : shape.getTerrainBoxes())
            {
               RigidBodyTransform boxPose = new RigidBodyTransform(pose);
               boxPose.appendTranslation(box.x(), box.y(), box.z());
               terrain.addRotatableBox(boxPose, box.sizeX(), box.sizeY(), box.sizeZ(), YoAppearance.LightSkyBlue());
            }
            for (TerrainSurface surface : shape.getTerrainSurfaces())
            {
               ConvexPolytope3D solid = new ConvexPolytope3D();
               for (Point3D vertex : surface.vertices())
               {
                  solid.addVertex(vertex);
                  solid.addVertex(new Point3D(vertex.getX(), vertex.getY(), surface.bottomZ()));
               }
               solid.applyTransform(pose);
               terrain.addTerrainObject(new ConvexPolytopeTerrainObject(solid, YoAppearance.LightSkyBlue()));
            }
         }
      }
      return terrain;
   }
}
