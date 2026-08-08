package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;

/**
 * Shared box-placement helpers for terrain objects built by stacking many {@code RotatableBoxTerrainObject}s onto a
 * {@link CombinedTerrainObject3D} (e.g. {@link PyramidStairsTerrainObject}, {@link InvertedPyramidStairsTerrainObject},
 * {@link RandomGridTerrainObject}).
 */
class BoxStackingTerrainTools
{
   private BoxStackingTerrainTools()
   {
   }

   /**
    * Adds a box to {@code terrain}, positioned at {@code (localX, localY, localZ)} in the local frame defined by
    * {@code transform} (i.e. the box's world pose is {@code transform} with an appended local translation).
    */
   static void addLocalBox(CombinedTerrainObject3D terrain, RigidBodyTransformReadOnly transform, double localX, double localY, double localZ,
                            double sizeX, double sizeY, double sizeZ, AppearanceDefinition appearance)
   {
      RigidBodyTransform pose = new RigidBodyTransform(transform);
      pose.appendTranslation(localX, localY, localZ);
      terrain.addRotatableBox(new Box3D(pose, sizeX, sizeY, sizeZ), appearance);
   }

   /**
    * Adds a flat, centered picture-frame border of the given {@code thickness} around a rectangular hole, in the
    * local frame defined by {@code transform}: the outer edge spans {@code outerSizeX x outerSizeY}, and the inner
    * (unfilled) hole spans {@code innerSizeX x innerSizeY}.
    */
   static void addFlatBorder(CombinedTerrainObject3D terrain, RigidBodyTransformReadOnly transform, double outerSizeX, double outerSizeY,
                              double innerSizeX, double innerSizeY, double thickness, double centerZ, AppearanceDefinition appearance)
   {
      double bandX = (outerSizeX - innerSizeX) / 2.0;
      double bandY = (outerSizeY - innerSizeY) / 2.0;

      // Top/bottom bands span the full outer width (including corners); left/right bands fill in the remaining
      // inner height between them, so the four pieces tile the frame with no gaps or overlaps.
      addLocalBox(terrain, transform, 0.0, (innerSizeY + bandY) / 2.0, centerZ, outerSizeX, bandY, thickness, appearance);
      addLocalBox(terrain, transform, 0.0, -(innerSizeY + bandY) / 2.0, centerZ, outerSizeX, bandY, thickness, appearance);
      addLocalBox(terrain, transform, (innerSizeX + bandX) / 2.0, 0.0, centerZ, bandX, innerSizeY, thickness, appearance);
      addLocalBox(terrain, transform, -(innerSizeX + bandX) / 2.0, 0.0, centerZ, bandX, innerSizeY, thickness, appearance);
   }
}
