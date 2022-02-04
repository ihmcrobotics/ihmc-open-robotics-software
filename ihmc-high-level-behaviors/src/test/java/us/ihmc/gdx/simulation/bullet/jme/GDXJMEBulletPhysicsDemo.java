package us.ihmc.gdx.simulation.bullet.jme;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.File;

public class GDXJMEBulletPhysicsDemo
{
   String directoryNameToAssumePresent = "ihmc-open-robotics-software";
   String subsequentPathToResourceFolder = "ihmc-high-level-behaviors/src/test/resources";
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), directoryNameToAssumePresent, subsequentPathToResourceFolder);

   public GDXJMEBulletPhysicsDemo()
   {
      File nativesDirectory = WorkspacePathTools.findPathToResource(directoryNameToAssumePresent,
                                                                    "ihmc-high-level-behaviors/src/test/resources",
                                                                    "libbulletjme")
                                                .toFile();
      NativeLibraryLoader.loadLibbulletjme(true, nativesDirectory, "Release", "Sp");

      PhysicsSpace.BroadphaseType bPhase = PhysicsSpace.BroadphaseType.DBVT;
      PhysicsSpace space = new PhysicsSpace(bPhase);

      float planeY = -1;
      Plane plane = new Plane(Vector3f.UNIT_Y, planeY);
      CollisionShape planeShape = new PlaneCollisionShape(plane);
      float mass = PhysicsBody.massForStatic;
      PhysicsRigidBody floor = new PhysicsRigidBody(planeShape, mass);
      space.addCollisionObject(floor);

      float radius = 0.3f;
      CollisionShape ballShape = new SphereCollisionShape(radius);
      mass = 1f;
      PhysicsRigidBody ball = new PhysicsRigidBody(ballShape, mass);
      space.addCollisionObject(ball);

      float timeStep = 0.02f;
      Vector3f location = new Vector3f();
      for (int i = 0; i < 50; ++i) {
         space.update(timeStep, 0);
         ball.getPhysicsLocation(location);
         System.out.println(location);
      }

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
         }

         @Override
         public void render()
         {

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXJMEBulletPhysicsDemo();
   }
}
