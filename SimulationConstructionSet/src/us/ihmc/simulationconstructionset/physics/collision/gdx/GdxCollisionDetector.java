package us.ihmc.simulationconstructionset.physics.collision.gdx;

import java.util.ArrayList;
import java.util.List;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.btAxisSweep3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShape;
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

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.CollisionShapeWithLink;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleContactWrapper;

public class GdxCollisionDetector implements ScsCollisionDetector
{
   private List<BulletCollisionShapeWithLink> allShapes = new ArrayList<BulletCollisionShapeWithLink>();
   private final btCollisionWorld collisionWorld;

   private final GdxCollisionFactory factory = new GdxCollisionFactory();

   private RigidBodyTransform transformScs = new RigidBodyTransform();
   private Matrix4 transformGdx = new Matrix4();

   static
   {
      Bullet.init();
   }

   /**
    *
    * @param worldRadius Sets the size of the world for broadphase collision detection.  Should be large enough to contain all the objects
    */
   public GdxCollisionDetector(double worldRadius)
   {
      btDefaultCollisionConfiguration collisionConfiguration = new btDefaultCollisionConfiguration();
      btCollisionDispatcher dispatcher = new btCollisionDispatcher(collisionConfiguration);

      float r = (float) worldRadius;
      Vector3 worldAabbMin = new Vector3(-r, -r, -r);
      Vector3 worldAabbMax = new Vector3(r, r, r);

      btAxisSweep3 broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

      collisionWorld = new btCollisionWorld(dispatcher, broadphase, collisionConfiguration);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public CollisionShapeFactory getShapeFactory()
   {
      return factory;
   }

   @Override
   public void performCollisionDetection(CollisionDetectionResult result)
   {
      Vector3D world = new Vector3D();

      for (int i = 0; i < allShapes.size(); i++)
      {
         BulletCollisionShapeWithLink info = allShapes.get(i);
         info.getTransformToWorld(transformScs);

         transformScs.getTranslation(world);

         GdxUtil.convert(transformScs, transformGdx);
         //         info.setTransformToWorld(transformScs);
         info.setWorldTransform(transformGdx);
      }

      collisionWorld.performDiscreteCollisionDetection();

      int numManifolds = collisionWorld.getDispatcher().getNumManifolds();
      for (int i = 0; i < numManifolds; i++)
      {
         btPersistentManifold contactManifold = collisionWorld.getDispatcher().getManifoldByIndexInternal(i);
         BulletCollisionShapeWithLink obA = (BulletCollisionShapeWithLink) contactManifold.getBody0();
         BulletCollisionShapeWithLink obB = (BulletCollisionShapeWithLink) contactManifold.getBody1();

         SimpleContactWrapper simpleContact = new SimpleContactWrapper(obA, obB);

         int numContacts = contactManifold.getNumContacts();
         for (int j = 0; j < numContacts; j++)
         {
            btManifoldPoint contactPoint = contactManifold.getContactPoint(j);

            Point3D pointOnA = new Point3D();
            Point3D pointOnB = new Point3D();

            Vector3 a = new Vector3();
            contactPoint.getPositionWorldOnA(a);

            Vector3 b = new Vector3();
            contactPoint.getPositionWorldOnB(b);

            GdxUtil.convert(a, pointOnA);
            GdxUtil.convert(b, pointOnB);

            float distance = contactPoint.getDistance();

            Vector3 v = new Vector3();

            contactPoint.getNormalWorldOnB(v);

            Vector3D normalA = new Vector3D();
            normalA.set(-v.x, -v.y, -v.z);

            simpleContact.addContact(pointOnA, pointOnB, normalA, distance);
         }

         result.addContact(simpleContact);

         // you can un-comment out this line, and then all points are removed
         //         contactManifold.clearManifold();
      }
   }

   public class GdxCollisionFactory implements CollisionShapeFactory
   {
      float margin = (float) CollisionShapeFactory.DEFAULT_MARGIN;

      @Override
      public void setMargin(double margin)
      {
         this.margin = (float) margin;
      }

      @Override
      public CollisionShapeDescription createBox(double radiusX, double radiusY, double radiusZ)
      {
         btBoxShape box = new btBoxShape(new Vector3((float) radiusX, (float) radiusY, (float) radiusZ));
         box.setMargin(margin);

         return new BulletShapeDescription(box);
      }

      @Override
      public CollisionShapeDescription createCylinder(double radius, double height)
      {
         btCylinderShape shape = new btCylinderShapeZ(new Vector3((float) radius, (float) radius, (float) height / 2.0f));
         shape.setMargin((float) margin);

         return new BulletShapeDescription(shape);
      }

      @Override
      public CollisionShapeDescription createSphere(double radius)
      {
         btSphereShape shape = new btSphereShape((float) radius);
         shape.setMargin(margin);

         return new BulletShapeDescription(shape);
      }

      @Override
      public CollisionShapeDescription createCapsule(double radius, double height)
      {
         btCapsuleShape shape = new btCapsuleShape((float) radius, (float) (height));
         shape.setMargin(margin);

         return new BulletShapeDescription(shape);
      }

      @Override
      public CollisionShape addShape(CollisionShapeDescription description)
      {
         RigidBodyTransform shapeToLink = new RigidBodyTransform();
         Link link = null;
         boolean isGround = false;

         BulletCollisionShapeWithLink shape = new BulletCollisionShapeWithLink("shape" + allShapes.size(), (BulletShapeDescription) description, link, isGround,
               shapeToLink);
         collisionWorld.addCollisionObject(shape, (short) 0xFFFF, (short) 0xFFFF);

         allShapes.add(shape);

         return shape;
      }

      @Override
      public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription description, boolean isGround, int collisionGroup,
            int collisionMask)
      {
         if (shapeToLink == null)
         {
            shapeToLink = new RigidBodyTransform();
         }

         BulletCollisionShapeWithLink shape = new BulletCollisionShapeWithLink("shape" + allShapes.size(), (BulletShapeDescription) description, link, isGround,
               shapeToLink);
         collisionWorld.addCollisionObject(shape, (short) collisionGroup, (short) collisionMask);

         allShapes.add(shape);

         return shape;
      }

      @Override
      public void addCollisionMeshDescription(Link link, CollisionMeshDescription collisionMeshDescription)
      {
         // TODO Auto-generated method stub
      }


   }

   /**
    * Just a wrapper around {@link com.bulletphysics.collision.shapes.CollisionShape}.
    */
   public static class BulletShapeDescription<T extends BulletShapeDescription<T>> implements CollisionShapeDescription<T>
   {
      private btCollisionShape shape;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

      public BulletShapeDescription(btCollisionShape shape)
      {
         this.shape = shape;
      }

      public btCollisionShape getShape()
      {
         return shape;
      }

      public void getTransform(RigidBodyTransform transformToPack)
      {
         transformToPack.set(transform);
      }

      @Override
      public BulletShapeDescription<T> copy()
      {
         BulletShapeDescription<T> copy = new BulletShapeDescription<T>(shape);
         return copy;
      }

      @Override
      public void setFrom(T collisionShapeDescription)
      {
         this.shape = collisionShapeDescription.getShape();
         collisionShapeDescription.getTransform(this.transform);
      }

      @Override
      public void applyTransform(RigidBodyTransform transformToWorld)
      {
         transform.preMultiply(transformToWorld);
      }

      @Override
      public void getBoundingBox(BoundingBox3d boundingBoxToPack)
      {
         boundingBoxToPack.set(boundingBox);
      }

      @Override
      public boolean isPointInside(Point3D pointInWorld)
      {
         throw new RuntimeException("Implement me!");
      }

      @Override
      public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
      {
         throw new RuntimeException("Implement me!");         
      }
   }

   /**
    * Reference to the shape's description and its link.
    */
   private static class BulletCollisionShapeWithLink extends btCollisionObject implements CollisionShapeWithLink
   {
      private final BulletShapeDescription description;
      private final Link link;
      private boolean isGround;

      // transform from shapeToLink coordinate system
      private final RigidBodyTransform shapeToLink = new RigidBodyTransform();
      private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

      private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

      public BulletCollisionShapeWithLink(String name, BulletShapeDescription description, Link link, boolean isGround, RigidBodyTransform shapeToLink)
      {
         this.description = description;
         this.link = link;
         this.isGround = isGround;

         this.shapeToLink.set(shapeToLink);
         setCollisionFlags(CollisionFlags.CF_KINEMATIC_OBJECT);
         setCollisionShape(description.shape);
      }

      private final RigidBodyTransform tempTransform = new RigidBodyTransform();

      @Override
      public CollisionShapeDescription getCollisionShapeDescription()
      {
         return description;
      }

      @Override
      public Link getLink()
      {
         return link;
      }

      @Override
      public boolean isGround()
      {
         return isGround;
      }

      @Override
      public void getShapeToLink(RigidBodyTransform shapeToLinkToPack)
      {
         shapeToLinkToPack.set(shapeToLink);
      }

      @Override
      public int getCollisionGroup()
      {
         return getBroadphaseHandle().getCollisionFilterGroup() & 0xFFFF;
      }

      @Override
      public int getCollisionMask()
      {
         return getBroadphaseHandle().getCollisionFilterMask() & 0xFFFF;
      }

      @Override
      public void getTransformToWorld(RigidBodyTransform transformToWorldToPack)
      {
         if (link != null)
         {
            link.getParentJoint().getTransformToWorld(tempTransform);
            transformToWorldToPack.set(tempTransform);
            transformToWorldToPack.multiply(shapeToLink);
         }
         else
         {
            transformToWorldToPack.set(transformToWorld);
         }
      }

      @Override
      public void setTransformToWorld(RigidBodyTransform transformToWorld)
      {
         if (link != null)
         {
            throw new RuntimeException("Shouldn't call this! Transform will be computed from the link location....");
         }
         else
         {
            this.transformToWorld.set(transformToWorld);
         }
      }

      @Override
      public CollisionShapeDescription getTransformedCollisionShapeDescription()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public void computeTransformedCollisionShape()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void setIsGround(boolean isGround)
      {
         this.isGround = isGround;
      }

      @Override
      public void setCollisionGroup(int groupMask)
      {
         // TODO Auto-generated method stub
         
      }

      @Override
      public void setCollisionMask(int collisionMask)
      {
         // TODO Auto-generated method stub
         
      }

      @Override
      public void getBoundingBox(BoundingBox3d boundingBoxToPack)
      {
         boundingBoxToPack.set(boundingBox);
      }
   }

}
