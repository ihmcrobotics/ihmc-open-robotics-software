package us.ihmc.rdx.simulation.bullet.libgdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.Random;

public class RDXLibGDXBulletPhysicsDemo2
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImBoolean simulate = new ImBoolean(false);
   private btCollisionConfiguration collisionConfiguration;
   private btCollisionDispatcher collisionDispatcher;
   private btBroadphaseInterface broadphase;
   private btConstraintSolver solver;
   private btDiscreteDynamicsWorld discreteDynamicsWorld;
   private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
   private final Vector3 tempVector = new Vector3();
   private btCollisionObject groundCollisionObject;

   public RDXLibGDXBulletPhysicsDemo2()
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            collisionConfiguration = new btDefaultCollisionConfiguration();
            collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
            broadphase = new btDbvtBroadphase();
            solver = new btSequentialImpulseConstraintSolver();
            discreteDynamicsWorld = new btDiscreteDynamicsWorld(collisionDispatcher, broadphase, solver, collisionConfiguration);
            Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
            discreteDynamicsWorld.setGravity(gravity);

            float groundSizeX = 1000.0f;
            float groundSizeY = 1000.0f;
            float groundSizeZ = 0.5f;
            ModelInstance groundModelInstance = RDXModelBuilder.createBox(groundSizeX, groundSizeY, groundSizeZ, Color.DARK_GRAY);
            RigidBodyTransform groundTransform = new RigidBodyTransform(new YawPitchRoll(0.0,
                                                                                         Math.toRadians(15.0),
                                                                                         0.0),
                                                                        new Point3D(0.0, 0.0, -0.0));
            groundTransform.appendTranslation(0.0, 0.0, -0.25);
            LibGDXTools.toLibGDX(groundTransform, groundModelInstance.transform);
            tempVector.set(groundSizeX / 2.0f, groundSizeY / 2.0f, groundSizeZ / 2.0f);
            btBoxShape groundBoxShape = new btBoxShape(tempVector);
            groundCollisionObject = new btCollisionObject();
            groundCollisionObject.setCollisionShape(groundBoxShape);
            groundCollisionObject.setWorldTransform(groundModelInstance.transform);
            discreteDynamicsWorld.addCollisionObject(groundCollisionObject);
            baseUI.getPrimaryScene().addModelInstance(groundModelInstance);

            int numberOfBlocks = 6;
            Random random = new Random(1886L);
            double x = 0.0;
            double y = 0.0;
            for (int i = 0; i < numberOfBlocks; i++)
            {
               float boxSizeX = 0.1f;
               float boxSizeY = 0.08f;
               float boxSizeZ = 0.1f;
               x += 0.02;
               y = 0.0;
               double z = boxSizeZ * 1.05 * (i + 1.0);
               ModelInstance boxModelInstance = RDXModelBuilder.createBox(boxSizeX, boxSizeY, boxSizeZ, Color.RED);
               double yaw = 0.0;
               double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
               double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
               RigidBodyTransform boxTransform = new RigidBodyTransform(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z));
               LibGDXTools.toLibGDX(boxTransform, boxModelInstance.transform);

               tempVector.set(boxSizeX / 2.0f, boxSizeY / 2.0f, boxSizeZ / 2.0f);
               btBoxShape boxShape = new btBoxShape(tempVector);
               float mass = 0.2f;
               Vector3 localInertia = new Vector3();
               boxShape.calculateLocalInertia(mass, localInertia);
               btMotionState motionState = new btMotionState()
               {
                  @Override
                  public void getWorldTransform(Matrix4 worldTrans)
                  {
                     worldTrans.set(boxModelInstance.transform);
                  }

                  @Override
                  public void setWorldTransform(Matrix4 worldTrans)
                  {
                     boxModelInstance.transform.set(worldTrans);
                  }
               };
               btRigidBody rigidBody = new btRigidBody(mass, motionState, boxShape, localInertia);
               rigidBodies.add(rigidBody);
               discreteDynamicsWorld.addRigidBody(rigidBody);

               baseUI.getPrimaryScene().addModelInstance(boxModelInstance);
            }

            baseUI.getImGuiPanelManager().addPanel("libGDX Bullet Physics", this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            if (simulate.get())
            {
               int maxSubSteps = 5;
               float fixedTimeStep = 1.0f / 60.0f;
               discreteDynamicsWorld.stepSimulation(Gdx.graphics.getDeltaTime(), maxSubSteps, fixedTimeStep);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox("Simulate", simulate);
         }

         @Override
         public void dispose()
         {
            discreteDynamicsWorld.removeCollisionObject(groundCollisionObject);
            for (btRigidBody rigidBody : rigidBodies)
            {
               discreteDynamicsWorld.removeRigidBody(rigidBody);
            }

            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXLibGDXBulletPhysicsDemo2();
   }
}
