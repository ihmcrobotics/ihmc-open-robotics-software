package us.ihmc.rdx.simulation.environment.object.objects;

import java.util.ArrayList;
import java.util.Random;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.collision.BoundingBox;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.physics.CollidableVisualizer;

public class RDXRockObject extends RDXEnvironmentObject
{
   public static final String NAME = "Rock";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXRockObject.class);

   public RDXRockObject()
   {
      super(NAME, FACTORY);
      
      int pointsPerRock = 40;
      double rockBoundingBoxWidth = 0.5;
      double maxRockCentroidHeight = 0.3;
      float mass = 900f;
      long randomNumber = 1989L;
      
      CreateRockObject(pointsPerRock, rockBoundingBoxWidth, maxRockCentroidHeight, mass, randomNumber);
   }

   public RDXRockObject(int pointsPerRock, double rockBoundingBoxWidth, double maxRockCentroidHeight, float mass, long randomNumber)
   {
      super(NAME, FACTORY);
      
      CreateRockObject(pointsPerRock, rockBoundingBoxWidth, maxRockCentroidHeight, mass, randomNumber);
   }

   private void CreateRockObject(int pointsPerRock, double rockBoundingBoxWidth, double maxRockCentroidHeight, float mass, long randomNumber)
   {

      Random random = new Random(randomNumber);

      //ConvexPolytope3D convexPolytope = EuclidPolytopeFactories.newIcosahedron(0.5);
      ArrayList<Point3D> vertexPoints = new ArrayList<Point3D>();
      double[][] vertices = new double[pointsPerRock][3];

      for (int j = 0; j < pointsPerRock; j++)
      {
         vertices[j][0] = random.nextDouble() * rockBoundingBoxWidth - rockBoundingBoxWidth / 2.0;
         vertices[j][1] = random.nextDouble() * rockBoundingBoxWidth - rockBoundingBoxWidth / 2.0;
         vertices[j][2] = random.nextDouble() * maxRockCentroidHeight - maxRockCentroidHeight / 2.0;
      }

      for (double[] point : vertices)
      {
         Point3D point3d = new Point3D(point[0], point[1], point[2]);
         vertexPoints.add(point3d);
      }

      ConvexPolytope3D rock = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertexPoints));
      RigidBodyTransform convexPolytopeTransform = new RigidBodyTransform();
      convexPolytopeTransform.getTranslation().set(rock.getCentroid());
      rock.applyInverseTransform(convexPolytopeTransform);

      Model model = RDXModelBuilder.buildModel(meshBuilder -> meshBuilder.addMesh(CollidableVisualizer.newConvexPolytope3DMesh(rock), Color.GRAY));
      setRealisticModel(model);

      BoundingBox box = new BoundingBox();
      model.calculateBoundingBox(box);

      setMass(mass);
      Box3D collisionBox = new Box3D(box.getDepth(), box.getWidth(), box.getHeight());

      getBoundingSphere().setRadius(box.getWidth() / 3);

      setCollisionModel(meshBuilder ->
      {
         Color color = LibGDXTools.toLibGDX(YoAppearance.LightBlue());
         meshBuilder.addMesh(CollidableVisualizer.newConvexPolytope3DMesh(rock), getCenterOfMassInModelFrame(), color);
      });

//      btConvexHullShape convexHullShape = new btConvexHullShape();
//      Vector3 vector = new Vector3();
//      for (Vertex3D vertix : rock.getVertices())
//      {
//         vector.set((float) vertix.getX(), (float) vertix.getY(), (float) vertix.getZ());
//         convexHullShape.addPoint(vector);
//      }
//
//      setBtCollisionShape(convexHullShape);
      setCollisionGeometryObject(collisionBox);
   }
}
