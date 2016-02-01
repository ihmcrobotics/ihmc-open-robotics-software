package us.ihmc.simulationconstructionset.physics.collision.gdx;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsForceSensor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;

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
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btPersistentManifold;
import com.badlogic.gdx.physics.bullet.collision.btSphereShape;
import com.badlogic.gdx.physics.bullet.linearmath.btVector3;

/**
 * @author Peter Abeles
 */
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

   public void performCollisionDetection()
   {
//    for( int i = 0; i < allSensors.size(); i++ ) {
//            allSensors.get(i).reset();
//    }

      Vector3d world = new Vector3d();

      for (int i = 0; i < allShapes.size(); i++)
      {
         ShapeInfo info = allShapes.get(i);

//       Joint joint = info.link.getParentJoint();


         info.getTransformToWorldForPhysics(transformScs, workSpace);

         transformScs.get(world);

//       System.out.println("Collision shape translation "+world.x+" "+world.y+" "+world.z);

         GdxUtil.convert(transformScs, transformGdx);
         info.setWorldTransform(transformGdx);
      }

      handler.maintenanceBeforeCollisionDetection();

      collisionWorld.performDiscreteCollisionDetection();
//      ContactWrapper contact = new ContactWrapper();

      int numManifolds = collisionWorld.getDispatcher().getNumManifolds();
      for (int i = 0; i < numManifolds; i++)
      {
         btPersistentManifold contactManifold = collisionWorld.getDispatcher().getManifoldByIndexInternal(i);
         ShapeInfo obA = (ShapeInfo) contactManifold.getBody0();
         ShapeInfo obB = (ShapeInfo) contactManifold.getBody1();

         ContactWrapper contact = new ContactWrapper();

         contact.setPersistentManifold(contactManifold);
         handler.handle(obA, obB, contact);

         // you can un-comment out this line, and then all points are removed
         // contactManifold->clearManifold();
      }
      
      handler.maintenanceAfterCollisionDetection();
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
         btCylinderShape shape = new btCylinderShape(new Vector3((float) radius, (float) height / 2.0f, (float) 0.0));
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

      public ScsForceSensor addForceSensor(String name, CollisionShape shape, RigidBodyTransform sensorToShape)
      {
         return null;
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


   public class ContactWrapper implements Contacts
   {
      btPersistentManifold pm;

      Vector3d normal = new Vector3d();

      public void setPersistentManifold(btPersistentManifold pm)
      {
         this.pm = pm;
      }

      public int getNumContacts()
      {
         return pm.getNumContacts();
      }

      public Point3d getWorldA(int which, Point3d location)
      {
         if (location == null)
            location = new Point3d();

         btVector3 vectorA = pm.getContactPoint(which).getPositionWorldOnA();
         GdxUtil.convert(vectorA, location);

         return location;
      }

      public Point3d getWorldB(int which, Point3d location)
      {
         if (location == null)
            location = new Point3d();

         btVector3 vectorB = pm.getContactPoint(which).getPositionWorldOnB();
         GdxUtil.convert(vectorB, location);

         return location;
      }

      public double getDistance(int which)
      {
         return pm.getContactPoint(which).getDistance();
      }

      public Vector3d getWorldNormal(int which)
      {
         btVector3 v = pm.getContactPoint(which).getNormalWorldOnB();
         normal.set(v.x(), v.y(), v.z());

         return normal;
      }

      public boolean isNormalOnA()
      {
         return false;
      }
   }
}
