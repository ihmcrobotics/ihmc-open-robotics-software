package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.yo.ImPlotYoPlot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class RDXBulletPhysicsManager
{
   static
   {
      RDXBulletTools.ensureBulletInitialized();
   }
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btMultiBodyConstraintSolver solver;
   private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final ArrayList<btMultiBody> multiBodies = new ArrayList<>();
   private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>(); // static, massless
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean simulate = new ImBoolean(false);
   private final ImFloat simulationRate = new ImFloat(1.0f);
   private final int worldTicksPerSecond = 240;
   /**
    * The simulation dt of the bullet world in bullet world time. This is always constant and for us, in
    * a robotics setting, rather than a gaming setting, want higher accuracy, so we set this to 250 Hz
    * rather than maybe a typical setting of 60 Hz for simple games.
    */
   private final float fixedTimeStep = (float) UnitConversions.hertzToSeconds(worldTicksPerSecond);
   /**
    * Say we call bullet simulate and wait around for a while before calling it again.
    * The next time we call Bullet, it is going to try to catch back up by performing
    * several simulation ticks. If we wait like, 5 minutes, we surely don't want Bullet
    * to try and to 5 minutes of simulation in one tick. We probably never want it to do
    * more than like 1/4 of a second.
    */
   private final double longestTimeWedEverWantToLetBulletSimulate = 0.25;
   private final int maxSubSteps = (int) Math.round(worldTicksPerSecond * longestTimeWedEverWantToLetBulletSimulate);
   private final ArrayList<Runnable> postTickRunnables = new ArrayList<>();
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger ticksSimulated = new YoInteger("ticksSimulated", yoRegistry);
   private final Stopwatch timeSpentInSteppingStopwatch = new Stopwatch();
   private final YoDouble timeSpentInStepping = new YoDouble("timeSpentInStepping", yoRegistry);
   private final YoDouble timeSpentPerTick = new YoDouble("timeSpentPerTick", yoRegistry);
   private final YoDouble realtimePerformance = new YoDouble("realtimePerformance", yoRegistry);
   private final ImPlotYoPlot ticksSimulatedPlot = new ImPlotYoPlot(ticksSimulated);
   private final ImPlotYoPlot timeSpentInSteppingPlot = new ImPlotYoPlot(timeSpentInStepping, timeSpentPerTick);
   private final ImPlotYoPlot realtimePerformancePlot = new ImPlotYoPlot(realtimePerformance);

   private RDXBulletPhysicsDebugger debugger;

   public void create()
   {
      collisionConfiguration = new btDefaultCollisionConfiguration();
      collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
      broadphase = new btDbvtBroadphase();
      solver = new btMultiBodyConstraintSolver();
      multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
      Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
      multiBodyDynamicsWorld.setGravity(gravity);
      debugger = new RDXBulletPhysicsDebugger(multiBodyDynamicsWorld);

      // Note: Apparently you can't have both pre and post tick callbacks, so we'll just do with post
      new InternalTickCallback(multiBodyDynamicsWorld, false)
      {
         @Override
         public void onInternalTick(btDynamicsWorld dynamicsWorld, float timeStep)
         {
            for (Runnable postTickRunnable : postTickRunnables)
            {
               postTickRunnable.run();
            }
         }
      };
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
      int collisionGroup = 1; // group 1 is rigid and static bodies
      int collisionGroupMask = 1 + 2; // Allow interaction with group 2, which is multi bodies
      multiBodyDynamicsWorld.addRigidBody(rigidBody, collisionGroup, collisionGroupMask);
      rigidBodies.add(rigidBody);
      return rigidBody;
   }

   public btMultiBody addMultiBody(btMultiBody multiBody)
   {
      multiBodyDynamicsWorld.addMultiBody(multiBody);
      multiBodies.add(multiBody);
      return multiBody;
   }

   public void addMultiBodyCollisionShape(btMultiBodyLinkCollider collisionShape)
   {
      int collisionGroup = 2; // Multi bodies need to be in a separate collision group
      int collisionGroupMask = 1 + 2; // But allowed to interact with group 1, which is rigid and static bodies
      multiBodyDynamicsWorld.addCollisionObject(collisionShape, collisionGroup, collisionGroupMask);
   }

   public void addMultiBodyConstraint(btMultiBodyConstraint constraint)
   {
      // TODO: Need to manage/remove this?
      multiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
   }

   public void removeMultiBodyConstraint(btMultiBodyConstraint constraint)
   {
      multiBodyDynamicsWorld.removeMultiBodyConstraint(constraint);
   }

   public void addPostTickRunnable(Runnable postTickRunnable)
   {
      postTickRunnables.add(postTickRunnable);
   }

   /**
    * See the Bullet wiki article on this:
    * https://web.archive.org/web/20170708145909/http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_the_World
    */
   public void simulate(float timePassedSinceThisWasCalledLast)
   {
      debugger.update();

      ticksSimulated.set(0);
      timeSpentInStepping.set(0.0);
      if (simulate.get())
      {
         // We can simulate slower or faster in real world time by gaming the first parameter.
         // Bullet will think it needs to catch up more, or it will not do as much.
         timePassedSinceThisWasCalledLast *= simulationRate.get();
         timeSpentInSteppingStopwatch.start();
         // FIXME: May sometimes cause EXCEPTION_ACCESS_VIOLATION
         ticksSimulated.set(multiBodyDynamicsWorld.stepSimulation(timePassedSinceThisWasCalledLast, maxSubSteps, fixedTimeStep));
         timeSpentInStepping.set(timeSpentInSteppingStopwatch.totalElapsed());
         if (ticksSimulated.getValue() > 0)
            timeSpentPerTick.set(timeSpentInStepping.getValue() / ticksSimulated.getValue());
         else
            timeSpentPerTick.set(0);

         realtimePerformance.set((ticksSimulated.getValue() * fixedTimeStep) / timeSpentInStepping.getValue());
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
      ImGui.text("Bullet simulation @ " + worldTicksPerSecond + " Hz dt");
      ImGui.checkbox(labels.get("Simulate"), simulate);
      ImGui.sliderFloat(labels.get("Simulation rate"), simulationRate.getData(), 0.001f, 1.0f);
      ticksSimulatedPlot.render(simulate.get());
      timeSpentInSteppingPlot.render(simulate.get());
      realtimePerformancePlot.render(simulate.get());
      debugger.renderImGuiWidgets();
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      debugger.getVirtualRenderables(renderables, pool);
   }

   public void destroy()
   {
      postTickRunnables.clear();
      for (btCollisionObject collisionObject : collisionObjects)
      {
         multiBodyDynamicsWorld.removeCollisionObject(collisionObject);
      }
      for (btRigidBody rigidBody : rigidBodies)
      {
         multiBodyDynamicsWorld.removeRigidBody(rigidBody);
      }
      for (btMultiBody multiBody : multiBodies)
      {
         multiBodyDynamicsWorld.removeMultiBody(multiBody);
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

   public ImFloat getSimulationRate()
   {
      return simulationRate;
   }
}
