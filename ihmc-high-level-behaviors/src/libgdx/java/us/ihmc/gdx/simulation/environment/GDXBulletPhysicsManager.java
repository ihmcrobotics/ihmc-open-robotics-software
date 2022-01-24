package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.btConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btDiscreteDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.dynamics.btSequentialImpulseConstraintSolver;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;

import java.util.ArrayList;

public class GDXBulletPhysicsManager
{
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btConstraintSolver solver;
   private btDiscreteDynamicsWorld discreteDynamicsWorld;
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>(); // static, massless

   public void create()
   {
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphase = new btDbvtBroadphase();
      solver = new btSequentialImpulseConstraintSolver();
      discreteDynamicsWorld = new btDiscreteDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      discreteDynamicsWorld.setGravity(gravity);
   }

   public void addStaticObject(btCollisionShape collisionShape, Matrix4 transformToWorld)
   {
      btCollisionObject staticObject = new btCollisionObject();
      staticObject.setCollisionShape(collisionShape);
      staticObject.setWorldTransform(transformToWorld);
      discreteDynamicsWorld.addCollisionObject(staticObject);
      collisionObjects.add(staticObject);
   }

   public void addRigidBody(btCollisionShape collisionShape, float mass, Matrix4 transformToWorld)
   {
      Vector3 localInertia = new Vector3();
      collisionShape.calculateLocalInertia(mass, localInertia);
      btMotionState motionState = new btMotionState()
      {
         @Override
         public void getWorldTransform(Matrix4 worldTrans)
         {
            worldTrans.set(transformToWorld);
         }

         @Override
         public void setWorldTransform(Matrix4 worldTrans)
         {
            transformToWorld.set(worldTrans);
         }
      };
      btRigidBody rigidBody = new btRigidBody(mass, motionState, collisionShape, localInertia);
      discreteDynamicsWorld.addRigidBody(rigidBody);
      rigidBodies.add(rigidBody);
   }

   public void simulate(float timeStep)
   {
      int maxSubSteps = 0; // 0 means use variable time step
      float fixedTimeStep = 0.0f; // value not used for variable time step
      discreteDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
   }

   public void destroy()
   {
      for (btCollisionObject collisionObject : collisionObjects)
      {
         discreteDynamicsWorld.removeCollisionObject(collisionObject);
      }
      for (btRigidBody rigidBody : rigidBodies)
      {
         discreteDynamicsWorld.removeRigidBody(rigidBody);
      }
   }
}
