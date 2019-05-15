package us.ihmc.robotics.geometry;

import java.util.ArrayList;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PlanarRegionsListGenerator
{
   private PlanarRegionsList planarRegionsList = new PlanarRegionsList(new ArrayList<PlanarRegion>());

   private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();

   private int id = 0;

   public void setId(int id)
   {
      this.id = id;
   }

   public void addCubeReferencedAtCenter(double lengthX, double widthY, double heightZ)
   {
      RigidBodyTransformGenerator transformGeneratorTwo = new RigidBodyTransformGenerator(transformGenerator);

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.translate(0.0, 0.0, heightZ / 2.0);
      addRectangle(transformGeneratorTwo, lengthX, widthY);
      transformGeneratorTwo.translate(0.0, 0.0, -heightZ);
      transformGeneratorTwo.rotate(Math.PI, Axis.Y);
      addRectangle(transformGeneratorTwo, lengthX, widthY);
      transformGeneratorTwo.rotate(Math.PI, Axis.Y);

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.rotate(Math.PI / 2.0, Axis.Y);
      transformGeneratorTwo.translate(0.0, 0.0, lengthX / 2.0);
      addRectangle(transformGeneratorTwo, heightZ, widthY);
      transformGeneratorTwo.translate(0.0, 0.0, -lengthX);
      transformGeneratorTwo.rotate(Math.PI, Axis.Y);
      addRectangle(transformGeneratorTwo, heightZ, widthY);
      transformGeneratorTwo.rotate(Math.PI, Axis.Y);

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.rotate(Math.PI / 2.0, Axis.X);
      transformGeneratorTwo.translate(0.0, 0.0, widthY / 2.0);
      addRectangle(transformGeneratorTwo, lengthX, heightZ);
      transformGeneratorTwo.translate(0.0, 0.0, -widthY);
      transformGeneratorTwo.rotate(Math.PI, Axis.X);
      addRectangle(transformGeneratorTwo, lengthX, heightZ);
      transformGeneratorTwo.rotate(Math.PI, Axis.X);
   }

   public void addCubeReferencedAtBottomMiddle(double lengthX, double widthY, double heightZ)
   {
      translate(0.0, 0.0, heightZ / 2.0);
      addCubeReferencedAtCenter(lengthX, widthY, heightZ);
      translate(0.0, 0.0, -heightZ / 2.0);
   }

   public void addCubeReferencedAtBottomNegativeXEdgeCenter(double lengthX, double widthY, double heightZ)
   {
      translate(-lengthX / 2.0, 0.0, heightZ / 2.0);
      addCubeReferencedAtCenter(lengthX, widthY, heightZ);
      translate(lengthX / 2.0, 0.0, -heightZ / 2.0);
   }

   public void addCubeReferencedAtBottomNegativeXYCorner(double lengthX, double widthY, double heightZ)
   {
      double halfLengthX = lengthX / 2.0;
      double halfWidthY = widthY / 2.0;
      translate(halfLengthX, halfWidthY, 0.0);
      addCubeReferencedAtBottomMiddle(lengthX, widthY, heightZ);
      translate(-halfLengthX, -halfWidthY, 0.0);
   }

   public void addRampReferencedAtBottomMiddle(double lengthX, double widthY, double heightZ)
   {
      RigidBodyTransformGenerator transformGeneratorTwo = new RigidBodyTransformGenerator(transformGenerator);
      double slope = Math.atan2(heightZ, lengthX);

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.translate(lengthX / 2.0, 0.0, heightZ / 2.0);
      transformGeneratorTwo.rotate(-slope, Axis.Y);
      addRectangle(transformGeneratorTwo, EuclidGeometryTools.pythagorasGetHypotenuse(lengthX, heightZ), widthY);

      ConvexPolygon2D leftSide = new ConvexPolygon2D();
      leftSide.addVertex(0.0, 0.0);
      leftSide.addVertex(- lengthX, 0.0);
      leftSide.addVertex(- lengthX, heightZ);
      leftSide.update();

      ConvexPolygon2D rightSide = new ConvexPolygon2D();
      rightSide.addVertex(0.0, 0.0);
      rightSide.addVertex(lengthX, 0.0);
      rightSide.addVertex(lengthX, heightZ);
      rightSide.update();

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.translate(0.0, 0.5 * widthY, 0.0);
      transformGeneratorTwo.rotate(0.5 * Math.PI, Axis.X);
      transformGeneratorTwo.rotate(Math.PI, Axis.Y);
      addPolygon(transformGeneratorTwo, leftSide);

      transformGeneratorTwo.set(transformGenerator);
      transformGeneratorTwo.translate(0.0, -0.5 * widthY, 0.0);
      transformGeneratorTwo.rotate(0.5 * Math.PI, Axis.X);
      addPolygon(transformGeneratorTwo, rightSide);
   }

   public void addRectangleReferencedAtNegativeXYCorner(double lengthX, double widthY)
   {
      translate(lengthX / 2, widthY / 2, 0.0);
      addRectangle(lengthX, widthY);
      translate(-lengthX / 2, -widthY / 2, 0.0);
   }

   public void addRectangle(double lengthX, double widthY)
   {
      ConvexPolygon2D rectangle = createRectanglePolygon(lengthX, widthY);
      addPolygon(rectangle);
   }

   public void addPolygon(ConvexPolygon2D polygon)
   {
      PlanarRegion planarRegion = new PlanarRegion(transformGenerator.getRigidBodyTransformCopy(), polygon);
      planarRegion.setRegionId(id++);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   public void addPolygon(RigidBodyTransformGenerator transformGenerator, ConvexPolygon2D polygon)
   {
      PlanarRegion planarRegion = new PlanarRegion(transformGenerator.getRigidBodyTransformCopy(), polygon);
      planarRegion.setRegionId(id++);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   public void addPolygons(ArrayList<ConvexPolygon2D> polygons)
   {
      PlanarRegion planarRegion = new PlanarRegion(transformGenerator.getRigidBodyTransformCopy(), polygons);
      planarRegion.setRegionId(id++);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   private void addRectangle(RigidBodyTransformGenerator transformGenerator, double lengthX, double widthY)
   {
      ConvexPolygon2D rectangle = createRectanglePolygon(lengthX, widthY);
      PlanarRegion planarRegion = new PlanarRegion(transformGenerator.getRigidBodyTransformCopy(), rectangle);
      planarRegion.setRegionId(id++);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   private static ConvexPolygon2D createRectanglePolygon(double lengthX, double widthY)
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, -widthY / 2.0);
      convexPolygon.addVertex(lengthX / 2.0, -widthY / 2.0);
      convexPolygon.update();
      return convexPolygon;
   }

   public void translate(double x, double y, double z)
   {
      transformGenerator.translate(x, y, z);
   }

   public void translate(Tuple3DReadOnly translationVector)
   {
      transformGenerator.translate(translationVector);
   }

   public void identity()
   {
      transformGenerator.identity();
   }

   public void rotateEuler(Vector3D eulerAngles)
   {
      transformGenerator.rotateEuler(eulerAngles);
   }

   public void rotate(RotationMatrix rotation)
   {
      transformGenerator.rotate(rotation);
   }

   public void rotate(Quaternion rotation)
   {
      transformGenerator.rotate(rotation);
   }

   public void rotate(double rotationAngle, Axis axis)
   {
      transformGenerator.rotate(rotationAngle, axis);
   }

   public void setTransform(RigidBodyTransform transform)
   {
      transformGenerator.setTransform(transform);
   }

   public void translateThenRotate(RigidBodyTransform transform)
   {
      transformGenerator.translateThenRotate(transform);
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public void reset()
   {
      planarRegionsList.clear();
      transformGenerator.identity();
   }
}