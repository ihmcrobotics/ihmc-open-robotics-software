package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;

public class GDXDoorLeverHandleObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Lever Handle";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorLeverHandleObject.class);
   private final double mass;
   private final ModelInstance realisticModelInstance;
   private final btBoxShape boxShape;
   private final btGImpactMeshShape btGImpactMeshShape;
   private final btConvexHullShape btConvexHullShape;
   private final ArrayList<Vector3D> vertices = new ArrayList<>();
   private final ArrayList<Integer> triangleIndexes = new ArrayList<>();
   private btRigidBody btRigidBody;

   public GDXDoorLeverHandleObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj");

      double sizeX = 0.065;
      double sizeY = 0.14;
      double sizeZ = 0.065;
      mass = 0.7;
      RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
      collisionShapeOffset.getTranslation().add(-sizeX / 2.0, -0.04, 0.0);
      Sphere3D boundingSphere = new Sphere3D(0.2);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      }, getPascalCasedName() + "CollisionModel" + getObjectIndex());
      collisionGraphic.materials.get(0).set(new BlendingAttribute(true, 0.4f));
      RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
      create(realisticModel, collisionGraphic, collisionShapeOffset, wholeThingOffset, boundingSphere, collisionBox, collisionBox::isPointInside, mass);

//      boolean enableDynamicAabbTree = true;
//      btCompoundShape btCompoundShape = new btCompoundShape(enableDynamicAabbTree);

      realisticModelInstance = new ModelInstance(realisticModel);

      btConvexHullShape = new btConvexHullShape();
      Mesh mesh = realisticModelInstance.model.meshParts.get(0).mesh;
      FloatBuffer verticesBuffer = mesh.getVerticesBuffer();
//      for (int i = 0; i < mesh.getNumVertices(); i++)
//      {
//         Vector3 vertex = new Vector3(verticesBuffer.get(), verticesBuffer.get(), verticesBuffer.get());
//         verticesBuffer.get(); // nx
//         verticesBuffer.get(); // ny
//         verticesBuffer.get(); // nz
//         verticesBuffer.get(); // uvx
//         verticesBuffer.get(); // uvy
//         btConvexHullShape.addPoint(vertex);
//      }
//      btConvexHullShape.addPoint(new Vector3((float) -sizeX / 2.0f, (float) -sizeY / 2.0f, (float) -sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) -sizeX / 2.0f, (float) -sizeY / 2.0f, (float) sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) -sizeX / 2.0f, (float) sizeY / 2.0f, (float) -sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) -sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) sizeX / 2.0f, (float) -sizeY / 2.0f, (float) -sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) sizeX / 2.0f, (float) -sizeY / 2.0f, (float) sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) -sizeZ / 2.0f));
//      btConvexHullShape.addPoint(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f));

      btTriangleIndexVertexArray btTriangleIndexVertexArray = new btTriangleIndexVertexArray();
//      btTriangleIndexVertexArray.addMeshPart(realisticModel.meshParts.get(0));
      btIndexedMesh btIndexedMesh = new btIndexedMesh();
      btIndexedMesh.setVertices(mesh.getVerticesBuffer(), Float.BYTES * 8, mesh.getNumVertices(), 0);
      btIndexedMesh.setIndices(mesh.getIndicesBuffer(), 0, mesh.getNumIndices());
      btTriangleIndexVertexArray.addIndexedMesh(btIndexedMesh);
//      btIndexedMesh indexedMesh = btTriangleIndexVertexArray.getIndexedMesh(0);
//      int numVertices = indexedMesh.getNumVertices();
//      int numTriangles = indexedMesh.getNumTriangles();
//      ByteBuffer vertexBase = indexedMesh.getVertexBase();
//      ByteBuffer triangleIndexBase = indexedMesh.getTriangleIndexBase();
//      for (int i = 0; i < numVertices; i++)
//      {
//         vertices.add(new Vector3D(vertexBase.getFloat(), vertexBase.getFloat(), vertexBase.getFloat()));
//      }
//      for (int i = 0; i < numTriangles; i++)
//      {
//         triangleIndexes.add(triangleIndexBase.getInt());
//      }
      //      btTriangleIndexVertexArray.
      btGImpactMeshShape = new btGImpactMeshShape(btTriangleIndexVertexArray);
      btGImpactMeshShape.updateBound();

      boxShape = new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) sizeY / 2.0f, (float) sizeZ / 2.0f));
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {

//      btRigidBody = bulletPhysicsManager.addRigidBody(btConvexHullShape, (float) mass, getBulletMotionState());
      btRigidBody = bulletPhysicsManager.addRigidBody(btGImpactMeshShape, (float) mass, getBulletMotionState());
//      btRigidBody = bulletPhysicsManager.addRigidBody(boxShape, (float) mass, getBulletMotionState());
   }

   @Override
   public void removeFromBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      bulletPhysicsManager.removeCollisionObject(btRigidBody);
   }
}
