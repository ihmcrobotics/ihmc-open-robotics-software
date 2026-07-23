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
 * object to be measured against, instead of an empty point in space. Converted with
 * {@code assimp export "Layout(1).fbx" LeverLayout.glb} from
 * {@code IsaacLab/source/isaaclab_assets/data/ihmc/lever_sim/Layout(1).fbx}. The source FBX is
 * authored in centimeters (confirmed by a baked 0.3937 = 1/2.54 cm-to-inch correction assimp left on
 * specific named sub-parts like {@code amazon_10x10_fixture_plate}, a real off-the-shelf part - not a
 * generic top-level unit conversion), and assimp's plain {@code export} does not apply a global
 * meters conversion the way its FBX *importer* metadata suggests it might - so a {@code "scale":
 * [0.01, 0.01, 0.01]} was baked directly onto the exported glTF's root node (see scratch conversion
 * notes; re-derive with a GLB node-hierarchy walk using each mesh's accessor min/max if this file is
 * ever regenerated, not by trusting {@code assimp info}'s reported bounding box, which does not match
 * the actual exported node hierarchy's own bounds). That 0.01 correction produced a real but visually
 * too-small (roughly hand-sized) prop once actually seen live next to the robot in RDXSim, so the root
 * node's scale was bumped to {@code [0.03, 0.03, 0.03]} (3x) - a live-in-sim visual judgment call, not
 * a re-derivation of the source asset's true physical size. Ground-truth world-space bounds at this
 * 0.03 scale: about 0.66m x 0.60m x 0.33m, origin at the object's base (local min z = 0).
 * No collision/physics joint - purely visual, matching the "static visual mesh only" scope chosen for
 * the first pass. A future pass adding a real revolute-joint lever is a separate, bigger task
 * (translating the USD physics/joint schema into an actual simulated joint).
 */
public class RDXLeverLayoutObject extends RDXEnvironmentObject
{
   public static final String NAME = "Lever Layout";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLeverLayoutObject.class);

   public RDXLeverLayoutObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/leverLayout/LeverLayout.glb");
      setRealisticModel(realisticModel);

      // Measured from the glTF's own accessor bounds after the 0.03 scale correction (see class
      // javadoc) - visual only, no physics/collision response depends on this being exact.
      double sizeX = 0.66;
      double sizeY = 0.60;
      double sizeZ = 0.33;
      setMass(5.0f);
      getCollisionShapeOffset().getTranslation().add(sizeX / 2.0, 0, 0.0);
      getBoundingSphere().setRadius(1.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
                        {
                           Color color = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
                           meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
                           meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
                        });
      setCollisionGeometryObject(collisionBox);
   }
}
