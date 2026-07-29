package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;

/**
 * Static visual-only prop for the Cosmos/RDX lever manipulation task (see {@code alex/scripts/cosmos/README.md}) -
 * gives the real robot camera something real to look at and gives the Cosmos target point a real
 * object to be measured against, instead of an empty point in space. Converted with the Omniverse
 * {@code omni.kit.asset_converter} extension (run headless via {@code isaacsim.SimulationApp}, pumping
 * {@code simulation_app.update()} to drive the converter's async task - a bare
 * {@code asyncio.get_event_loop().run_until_complete()} silently collides with Kit's own already-running
 * loop and never executes) from
 * {@code /home/bpratt/IsaacLab-Arena/isaaclab_arena/assets/lever_sim/lever_finally.usd}, the successor to
 * the original {@code Layout(1).fbx} (still present alongside it in that directory; same underlying
 * sub-parts, e.g. {@code amazon_10x10_fixture_plate} with the same baked 0.3937 = 1/2.54 cm-to-inch
 * correction). World-space bounds were measured directly from the USD with
 * {@code UsdGeom.BBoxCache.ComputeWorldBound} (stage {@code metersPerUnit=0.0254}, i.e. authored in
 * inches): X=0.51m, Y=0.53m, Z=0.28m, origin at the object's base ({@code Z_min=0} in world space -
 * despite the stage's {@code upAxis} token reading "Y", the composed geometry's actual base-to-top axis
 * is Z, same as the old asset, so no axis remap was needed). No manual scale correction was required
 * this time - the asset-converter path (unlike the old assimp {@code export} path) round-trips the
 * stage's real-world scale correctly, and the result is close in magnitude to the old asset's
 * 0.66m x 0.60m x 0.33m, which is a good sanity check against a stray unit-conversion bug.
 * No collision/physics joint - purely visual, matching the "static visual mesh only" scope chosen for
 * the first pass. A future pass adding a real revolute-joint lever is a separate, bigger task
 * (translating the USD physics/joint schema into an actual simulated joint).
 */
public class RDXLeverLayoutObject extends RDXEnvironmentObject
{
   public static final String NAME = "Lever Layout";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLeverLayoutObject.class);
   private static final double COLLISION_SIZE_X = 0.25;
   private static final double COLLISION_SIZE_Y = 0.25;
   private static final double COLLISION_SIZE_Z = 0.25;

   private final Box3D collisionBox = new Box3D(COLLISION_SIZE_X, COLLISION_SIZE_Y, COLLISION_SIZE_Z);
   private double uniformScale = 1.0;

   public RDXLeverLayoutObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/leverLayout/lever_finally.glb");
      setRealisticModel(realisticModel);

      // Measured directly from the source USD's world-space bounding box (see class javadoc) - visual
      // only, no physics/collision response depends on this being exact.
      setMass(5.0f);
      getCollisionShapeOffset().getTranslation().add(COLLISION_SIZE_X / 2.0, 0, 0.0);
      getBoundingSphere().setRadius(1.0);
      getBoundingSphere().getPosition().add(COLLISION_SIZE_X / 2.0, COLLISION_SIZE_Y / 2.0, COLLISION_SIZE_Z / 2.0);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
                           meshBuilder.addBox((float) COLLISION_SIZE_X, (float) COLLISION_SIZE_Y, (float) COLLISION_SIZE_Z, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
                        });
      setCollisionGeometryObject(collisionBox);
   }

   /** Uniformly scales the visual model and its selection/collision proxy about the object's origin. */
   public void setUniformScale(double uniformScale)
   {
      this.uniformScale = Math.max(0.01, uniformScale);
      collisionBox.getSize().set(COLLISION_SIZE_X * this.uniformScale,
                                 COLLISION_SIZE_Y * this.uniformScale,
                                 COLLISION_SIZE_Z * this.uniformScale);
      getCollisionShapeOffset().getTranslation().set(COLLISION_SIZE_X * this.uniformScale / 2.0, 0.0, 0.0);
      getBoundingSphere().setRadius(this.uniformScale);
      updateRenderablesPoses();
   }

   public double getUniformScale()
   {
      return uniformScale;
   }

   @Override
   public void updateRenderablesPoses()
   {
      super.updateRenderablesPoses();
      float scale = (float) uniformScale;
      realisticModelInstance.transform.scale(scale, scale, scale);
      collisionModelInstance.transform.scale(scale, scale, scale);
   }
}
