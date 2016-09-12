package us.ihmc.simulationconstructionset.physics.collision.gdx;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.btAxisSweep3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionWorld;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShape;
import com.badlogic.gdx.physics.bullet.collision.btCylinderShapeZ;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.physics.bullet.collision.btPersistentManifold;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;


public class GdxCollisionDetector implements ScsCollisionDetector
{
   private CollisionHandler handler;
   private List<ShapeInfo> allShapes = new ArrayList<ShapeInfo>();
   private final btCollisionWorld collisionWorld;

   private final GdxCollisionFactory factory = new GdxCollisionFactory();

   private YoVariableRegistry registry;

   private RigidBodyTransform transformScs = new RigidBodyTransform();
   private Matrix4 transformGdx = new Matrix4();

   private RigidBodyTransform workSpace = new RigidBodyTransform();

   static
   {
      Bullet.init();
   }

   /**
    *
    * @param worldRadius Sets the size of the world for broadphase collision detection.  Should be large enough to contain all the objects
    */
   public GdxCollisionDetector(YoVariableRegistry registryParent, double worldRadius)
   {
      registry = new YoVariableRegistry("GDX");
      registryParent.addChild(registry);

      btDefaultCollisionConfiguration collisionConfiguration = new btDefaultCollisionConfiguration();
      btCollisionDispatcher dispatcher = new btCollisionDispatcher(collisionConfiguration);

      float r = (float) worldRadius;
      Vector3 worldAabbMin = new Vector3(-r, -r, -r);
      Vector3 worldAabbMax = new Vector3(r, r, r);


      btAxisSweep3 broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

      collisionWorld = new btCollisionWorld(dispatcher, broadphase, collisionConfiguration);
   }


   public void initialize(CollisionHandler handler)
   {
      this.handler = handler;
   }

   public CollisionShapeFactory getShapeFactory()
   {
      return factory;
   }

   public void removeShape(Link link)
   {
      ShapeInfo info = (ShapeInfo) link.getCollisionShape();
      collisionWorld.removeCollisionObject(info);
      allShapes.remove(info);
   }

   public CollisionShape lookupCollisionShape(Link link)
   {
      for (int i = 0; i < allShapes.size(); i++)
      {
         ShapeInfo info = allShapes.get(i);
         if (info.link == link)
            return info;
      }

      throw new RuntimeException("Can't find matching shape");
   }

   @Override
   public void performCollisionDetection(CollisionDetectionResult result)
   {
      Vector3d world = new Vector3d();

      for (int i = 0; i < allShapes.size(); i++)
      {
         ShapeInfo info = allShapes.get(i);
         info.getTransformToWorldForPhysics(transformScs, workSpace);

         transformScs.getTranslation(world);

//       System.out.println("Collision shape translation "+world.x+" "+world.y+" "+world.z);

         GdxUtil.convert(transformScs, transformGdx);
         info.setWorldTransform(transformGdx);
      }

      if (handler != null) handler.maintenanceBeforeCollisionDetection();

      collisionWorld.performDiscreteCollisionDetection();
//      ContactWrapper contact = new ContactWrapper();

      int numManifolds = collisionWorld.getDispatcher().getNumManifolds();
      for (int i = 0; i < numManifolds; i++)
      {
         btPersistentManifold contactManifold = collisionWorld.getDispatcher().getManifoldByIndexInternal(i);
         ShapeInfo obA = (ShapeInfo) contactManifold.getBody0();
         ShapeInfo obB = (ShapeInfo) contactManifold.getBody1();

         int numContacts = contactManifold.getNumContacts();
         for (int j=0; j<numContacts; j++)
         {
            btManifoldPoint contactPoint = contactManifold.getContactPoint(j);

            Point3d pointOnA = new Point3d();
            Point3d pointOnB = new Point3d();

            Vector3 a = new Vector3();
            contactPoint.getPositionWorldOnA(a);

            Vector3 b = new Vector3();
            contactPoint.getPositionWorldOnB(b);

            GdxUtil.convert(a, pointOnA);
            GdxUtil.convert(b, pointOnB);

//            System.out.println("contactPointOnA = " + pointOnA);
//            System.out.println("contactPointOnB = " + pointOnB);

            result.addContact(obA, obB, pointOnA, pointOnB);
         }


         ContactWrapper contact = new ContactWrapper();

         contact.setPersistentManifold(contactManifold);
         if (handler != null) handler.handle(obA, obB, contact);

         // you can un-comment out this line, and then all points are removed
         // contactManifold->clearManifold();
      }

      if (handler != null) handler.maintenanceAfterCollisionDetection();
   }

   public class GdxCollisionFactory implements CollisionShapeFactory
   {
      float margin = (float) CollisionShapeFactory.DEFAULT_MARGIN;

      public void setMargin(double margin)
      {
         this.margin = (float) margin;
      }

      public CollisionShapeDescription createBox(double radiusX, double radiusY, double radiusZ)
      {
         btBoxShape box = new btBoxShape(new Vector3((float) radiusX, (float) radiusY, (float) radiusZ));
         box.setMargin(margin);

         return new ShapeDescription(box);
      }

      public CollisionShapeDescription createCylinder(double radius, double height)
      {
         btCylinderShape shape = new btCylinderShapeZ(new Vector3((float) radius, (float) radius, (float) height / 2.0f));
         shape.setMargin((float) margin);

         return new ShapeDescription(shape);
      }

      public CollisionShapeDescription createSphere(double radius)
      {
         btSphereShape shape = new btSphereShape((float) radius);
         shape.setMargin(margin);

         return new ShapeDescription(shape);
      }

      public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription description, boolean isGround, int collisionGroup, int collisionMask)
      {
         if (shapeToLink == null)
         {
            shapeToLink = new RigidBodyTransform();
         }

         ShapeInfo shape = new ShapeInfo("shape" + allShapes.size(), (ShapeDescription) description, link, isGround, shapeToLink, registry);
         collisionWorld.addCollisionObject(shape, (short) collisionGroup, (short) collisionMask);

         allShapes.add(shape);

         return shape;
      }
   }


   /**
    * Just a wrapper around {@link com.bulletphysics.collision.shapes.CollisionShape}.
    */
   public static class ShapeDescription implements CollisionShapeDescription
   {
      public btCollisionShape shape;

      public ShapeDescription(btCollisionShape shape)
      {
         this.shape = shape;
      }
   }


   /**
    * Reference to the shape's description and its link.
    */
   public static class ShapeInfo extends btCollisionObject implements CollisionShape
   {
      private final ShapeDescription description;
      private final Link link;
      private final boolean isGround;

      // transform from shapeToLink coordinate system
      RigidBodyTransform shapeToLink = new RigidBodyTransform();

      YoVariableRegistry registry;

      public ShapeInfo(String name, ShapeDescription description, Link link, boolean isGround, RigidBodyTransform shapeToLink, YoVariableRegistry registryParent)
      {
         this.description = description;
         this.link = link;
         this.isGround = isGround;

         this.shapeToLink.set(shapeToLink);
         setCollisionFlags(CollisionFlags.CF_KINEMATIC_OBJECT);
         setCollisionShape(description.shape);

         registry = new YoVariableRegistry(name);
         registryParent.addChild(registry);
      }

      /**
       * Returns the transform from shape to world coordinates.  Used for physics simulation
       *
       * @param shapeToWorld The transform from shape reference frame to the world.
       */
      public void getTransformToWorldForPhysics(RigidBodyTransform shapeToWorld, RigidBodyTransform workSpace)
      {
         link.getParentJoint().getTransformToWorld(workSpace);

         shapeToWorld.multiply(workSpace, shapeToLink);
      }

      public CollisionShapeDescription getDescription()
      {
         return description;
      }

      public Link getLink()
      {
         return link;
      }

      public boolean isGround()
      {
         return isGround;
      }

      public RigidBodyTransform getShapeToLink()
      {
         return shapeToLink;
      }

      public int getGroupMask()
      {
         return getBroadphaseHandle().getCollisionFilterGroup() & 0xFFFF;
      }

      public int getCollisionMask()
      {
         return getBroadphaseHandle().getCollisionFilterMask() & 0xFFFF;
      }

      public double distance(double x, double y, double z)
      {
         return 0;
      }
   }


   private class ContactWrapper implements Contacts
   {
      private btPersistentManifold persistentManifold;

      private final Vector3d normal = new Vector3d();

      public void setPersistentManifold(btPersistentManifold pm)
      {
         this.persistentManifold = pm;
      }

      public int getNumContacts()
      {
         return persistentManifold.getNumContacts();
      }

      public Point3d getWorldA(int which, Point3d location)
      {
         if (location == null)
            location = new Point3d();

         Vector3 vectorA = new Vector3();
         persistentManifold.getContactPoint(which).getPositionWorldOnA(vectorA);
         GdxUtil.convert(vectorA, location);

         return location;
      }

      public Point3d getWorldB(int which, Point3d location)
      {
         if (location == null)
            location = new Point3d();

         Vector3 vectorB = new Vector3();
         persistentManifold.getContactPoint(which).getPositionWorldOnB(vectorB);
         GdxUtil.convert(vectorB, location);

         return location;
      }

      public double getDistance(int which)
      {
         return persistentManifold.getContactPoint(which).getDistance();
      }

      public Vector3d getWorldNormal(int which)
      {
         Vector3 v = new Vector3();

         persistentManifold.getContactPoint(which).getNormalWorldOnB(v);
         normal.set(v.x, v.y, v.z);

         return normal;
      }

      public boolean isNormalOnA()
      {
         return false;
      }
   }
}
