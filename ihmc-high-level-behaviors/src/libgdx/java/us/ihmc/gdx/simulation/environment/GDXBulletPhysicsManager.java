package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.simulation.GDXBulletPhysicsDebugger;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;

import java.util.ArrayList;

public class GDXBulletPhysicsManager
{
   static
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
   }
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btMultiBodyConstraintSolver solver;
   private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final ArrayList<btMultiBody> multiBodies = new ArrayList<>();
   private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>(); // static, massless
   private ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean simulate = new ImBoolean(false);
   private ImFloat simulationRate = new ImFloat(1.0f);
   private GDXBulletPhysicsDebugger debugger;

   public void create()
   {
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphase = new btDbvtBroadphase();
      solver = new btMultiBodyConstraintSolver();
      multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      multiBodyDynamicsWorld.setGravity(gravity);
      debugger = new GDXBulletPhysicsDebugger(multiBodyDynamicsWorld);
   }

   public btCollisionObject addStaticObject(btCollisionShape collisionShape, Matrix4 transformToWorld)
   {
      btCollisionObject staticObject = new btCollisionObject();
      staticObject.setCollisionShape(collisionShape);
      staticObject.setWorldTransform(transformToWorld);
      multiBodyDynamicsWorld.addCollisionObject(staticObject);
      collisionObjects.add(staticObject);
      return staticObject;
   }

   public btRigidBody addRigidBody(btCollisionShape collisionShape, float mass, Matrix4 transformToWorld)
   {
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
      return addRigidBody(collisionShape, mass, motionState);
   }

   public btRigidBody addRigidBody(btCollisionShape collisionShape, float mass, btMotionState motionState)
   {
      Vector3 localInertia = new Vector3();
      collisionShape.calculateLocalInertia(mass, localInertia);
      btRigidBody rigidBody = new btRigidBody(mass, motionState, collisionShape, localInertia);
      multiBodyDynamicsWorld.addRigidBody(rigidBody);
      rigidBodies.add(rigidBody);
      return rigidBody;
   }

   public btMultiBody addMultiBody(btMultiBody multiBody)
   {
      multiBodyDynamicsWorld.addMultiBody(multiBody);
      multiBodies.add(multiBody);
      return multiBody;
   }

   public void simulate(float timeStep)
   {
      debugger.update();
      if (simulate.get())
      {
         timeStep *= simulationRate.get();
         int maxSubSteps = 1000; // 0 means use variable time step
         float fixedTimeStep = (float) UnitConversions.hertzToSeconds(240); // value not used for variable time step
         multiBodyDynamicsWorld.stepSimulation(timeStep, maxSubSteps, fixedTimeStep); // FIXME: Sometimes EXCEPTION_ACCESS_VIOLATION
      }
   }

   public void setKinematicObject(btRigidBody btRigidBody, boolean isKinematicObject)
   {
      if (isKinematicObject)
      {
         btRigidBody.setCollisionFlags(btRigidBody.getCollisionFlags() | btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
         btRigidBody.setActivationState(CollisionConstants.DISABLE_DEACTIVATION);
      }
      else
      {
         btRigidBody.setCollisionFlags(btRigidBody.getCollisionFlags() & ~btCollisionObject.CollisionFlags.CF_KINEMATIC_OBJECT);
         btRigidBody.setActivationState(CollisionConstants.WANTS_DEACTIVATION);
      }
   }

   public void removeCollisionObject(btCollisionObject collisionObject)
   {
      multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      rigidBodies.remove(collisionObject);
   }

   public void removeMultiBody(btMultiBody btMultiBody)
   {
      multiBodyDynamicsWorld.removeMultiBody(btMultiBody);
      multiBodies.remove(btMultiBody);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Simulate with Bullet"), simulate);
      ImGui.sliderFloat(labels.get("Simulation rate"), simulationRate.getData(), 0.001f, 1.0f);
      debugger.renderImGuiWidgets();
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      debugger.getVirtualRenderables(renderables, pool);
   }

   public void destroy()
   {
      for (btCollisionObject collisionObject : collisionObjects)
      {
         multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      }
      for (btRigidBody rigidBody : rigidBodies)
      {
         // Can probably just call removeCollisionObject
         multiBodyDynamicsWorld.removeRigidBody(rigidBody);
      }
   }

   public ImBoolean getSimulate()
   {
      return simulate;
   }

   public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld()
   {
      return multiBodyDynamicsWorld;
   }
}
