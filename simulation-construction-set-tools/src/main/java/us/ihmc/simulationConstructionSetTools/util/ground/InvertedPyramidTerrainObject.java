package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

/**
 * A truncated (frustum) square pit/bowl terrain object &mdash; the mirror image of {@link PyramidTerrainObject}.
 * Modeled on IsaacLab's HfInvertedPyramidSlopedTerrain (isaac-sim/IsaacLab,
 * {@code terrains/height_field/hf_terrains.py}, {@code pyramid_sloped_terrain} with {@code cfg.inverted = True}).
 * <p>
 * The shape is defined by the same three parameters as {@link PyramidTerrainObject}:
 * <ul>
 * <li>{@code slope}: the rise over run of the sloped side walls.
 * <li>{@code baseWidth}: the side length of the square rim, which sits on the ground plane ({@code z = 0}).
 * <li>{@code topWidth}: the side length of the square flat platform at the bottom of the pit.
 * </ul>
 * The depth of the pit is derived from these: {@code depth = slope * (baseWidth - topWidth) / 2.0}, with the rim at
 * {@code z = 0} and the bottom platform at {@code z = -depth}. A {@link RigidBodyTransformReadOnly} may be supplied
 * to place and orient the pit anywhere in the world, which makes it convenient to add to a
 * {@link CombinedTerrainObject3D}.
 */
public class InvertedPyramidTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private static final double EPSILON = 1.0e-12;

   private final double depth;

   private final BoundingBox3D boundingBox = new BoundingBox3D();
   private final ConvexPolytope3D collisionShape;
   private final Graphics3DObject linkGraphics;

   public InvertedPyramidTerrainObject(double slope, double baseWidth, double topWidth, AppearanceDefinition appearance)
   {
      this(new RigidBodyTransform(), slope, baseWidth, topWidth, appearance);
   }

   public InvertedPyramidTerrainObject(RigidBodyTransformReadOnly transform, double slope, double baseWidth, double topWidth)
   {
      this(transform, slope, baseWidth, topWidth, YoAppearance.StoneTexture());
   }

   public InvertedPyramidTerrainObject(RigidBodyTransformReadOnly transform, double slope, double baseWidth, double topWidth,
                                        AppearanceDefinition appearance)
   {
      if (baseWidth <= 0.0)
         throw new IllegalArgumentException("baseWidth must be positive. Was: " + baseWidth);
      if (topWidth < 0.0)
         throw new IllegalArgumentException("topWidth must be non-negative. Was: " + topWidth);
      if (topWidth > baseWidth)
         throw new IllegalArgumentException("topWidth must not exceed baseWidth. topWidth: " + topWidth + ", baseWidth: " + baseWidth);
      if (slope <= 0.0)
         throw new IllegalArgumentException("slope must be positive. Was: " + slope);

      depth = slope * (baseWidth - topWidth) / 2.0;

      double halfBase = baseWidth / 2.0;
      double halfTop = topWidth / 2.0;

      // Rim vertices lie in the z = 0 plane; bottom-platform vertices at z = -depth. All in the local frame.
      Point3D[] rimVertices = {new Point3D(-halfBase, -halfBase, 0.0),
                               new Point3D(halfBase, -halfBase, 0.0),
                               new Point3D(halfBase, halfBase, 0.0),
                               new Point3D(-halfBase, halfBase, 0.0)};
      Point3D[] bottomVertices = {new Point3D(-halfTop, -halfTop, -depth),
                                  new Point3D(halfTop, -halfTop, -depth),
                                  new Point3D(halfTop, halfTop, -depth),
                                  new Point3D(-halfTop, halfTop, -depth)};

      List<Point3D> vertices = new ArrayList<>();
      for (Point3D vertex : rimVertices)
      {
         vertex.applyTransform(transform);
         vertices.add(vertex);
      }
      for (Point3D vertex : bottomVertices)
      {
         vertex.applyTransform(transform);
         vertices.add(vertex);
      }

      collisionShape = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));
      boundingBox.set(collisionShape.getBoundingBox());

      linkGraphics = new Graphics3DObject();
      addLinkGraphics(rimVertices, bottomVertices, appearance);
   }

   private void addLinkGraphics(Point3D[] rimVertices, Point3D[] bottomVertices, AppearanceDefinition appearance)
   {
      // Four trapezoidal side (wall) faces, ordered so the outward normal points away from the pit (i.e. up/out).
      for (int i = 0; i < 4; i++)
      {
         int next = (i + 1) % 4;
         Point3D[] sideFace = {new Point3D(rimVertices[i]), new Point3D(rimVertices[next]), new Point3D(bottomVertices[next]), new Point3D(bottomVertices[i])};
         linkGraphics.addPolygon(sideFace, appearance);
      }

      // Bottom (floor) face.
      Point3D[] bottomFace = {new Point3D(bottomVertices[0]), new Point3D(bottomVertices[1]), new Point3D(bottomVertices[2]), new Point3D(bottomVertices[3])};
      linkGraphics.addPolygon(bottomFace, appearance);
   }

   public double getDepth()
   {
      return depth;
   }

   private Face3DReadOnly highestFaceUnderVerticalLine(double x, double y)
   {
      Point3D pointOnLine = new Point3D(x, y, 0.0);
      double highest = Double.NEGATIVE_INFINITY;
      Face3DReadOnly highestFace = null;

      for (Face3DReadOnly face : collisionShape.getFaces())
      {
         Point3D intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(face.getCentroid(), face.getNormal(), pointOnLine, Axis3D.Z);

         if (intersection != null && face.distance(intersection) <= EPSILON && intersection.getZ() > highest)
         {
            highest = intersection.getZ();
            highestFace = face;
         }
      }

      return highestFace;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      Face3DReadOnly highestFace = highestFaceUnderVerticalLine(x, y);
      if (highestFace == null)
         return Double.NEGATIVE_INFINITY;

      Point3D intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(highestFace.getCentroid(),
                                                                                     highestFace.getNormal(),
                                                                                     new Point3D(x, y, 0.0),
                                                                                     Axis3D.Z);
      return intersection.getZ();
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      Face3DReadOnly highestFace = highestFaceUnderVerticalLine(x, y);
      if (highestFace == null)
         return Double.NEGATIVE_INFINITY;

      if (normalToPack != null)
         normalToPack.set(highestFace.getNormal());

      Point3D intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(highestFace.getCentroid(),
                                                                                     highestFace.getNormal(),
                                                                                     new Point3D(x, y, 0.0),
                                                                                     Axis3D.Z);
      return intersection.getZ();
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      Face3DReadOnly highestFace = highestFaceUnderVerticalLine(x, y);
      double heightAt = Double.NEGATIVE_INFINITY;

      if (highestFace != null)
      {
         Point3D intersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(highestFace.getCentroid(),
                                                                                        highestFace.getNormal(),
                                                                                        new Point3D(x, y, 0.0),
                                                                                        Axis3D.Z);
         heightAt = intersection.getZ();
      }

      if (intersectionToPack != null)
         intersectionToPack.set(x, y, heightAt);

      if (normalToPack != null && highestFace != null)
         normalToPack.set(highestFace.getNormal());

      return z < heightAt;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return Collections.singletonList(collisionShape);
   }
}
