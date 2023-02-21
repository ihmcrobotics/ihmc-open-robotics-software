package us.ihmc.rdx.simulation.bullet.libgdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.physics.CollidableVisualizer;

import java.util.Random;
public class RDXLibGDXBulletPhysicsDemo4
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private BulletWorld world;
   private final ImBoolean simulate = new ImBoolean(false);

   public RDXLibGDXBulletPhysicsDemo4()
   {
      Bullet.init();
      LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
            world = new BulletWorld(gravity);

            ModelInstance ground = RDXModelBuilder.createBox(1000.0f, 1000.0f, 0.5f, Color.DARK_GRAY);
            double polytopeSizeZ = 0.5;
            ConvexPolytope3D convexPolytope = EuclidPolytopeFactories.newIcosahedron(0.5);
            Model model = RDXModelBuilder.buildModel(meshBuilder -> meshBuilder.addMesh(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), Color.GRAY));
            ModelInstance poloytope = new ModelInstance(model);

            world.addConstructor("ground", new BulletConstructor(ground.model, 0.0f));
            world.addConstructor("poloytope", new BulletConstructor(poloytope.model, 0.9f));
            RigidBodyTransform groundTransform = new RigidBodyTransform(new YawPitchRoll(0.0,
                                                                                         Math.toRadians(15.0),
                                                                                         0.0),
                                                                        new Point3D(0.0, 0.0, -0.0));
            groundTransform.appendTranslation(0.0, 0.0, -0.25);
            Matrix4 groundTransformGDX = new Matrix4();
            LibGDXTools.toLibGDX(groundTransform, groundTransformGDX);
            world.add("ground", groundTransformGDX);

            int numberOfPolytopes = 6;
            Random random = new Random(1886L);
            double x = 0.0;
            double y = 0.0;
            for (int i = 0; i < numberOfPolytopes; i++)
            {
               x += 0.02;
               y = 0.0;
               double z = polytopeSizeZ * 1.75 * (i + 1.0);

               double yaw = 0.0;
               double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
               double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

               RigidBodyTransform polytopeTransform = new RigidBodyTransform(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z));
               Matrix4 polytopeTransformGDX = new Matrix4();
               LibGDXTools.toLibGDX(polytopeTransform, polytopeTransformGDX);
               world.add("poloytope", polytopeTransformGDX);
            }
            
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               for (BulletEntity entity : world.getEntities())
               {
                  entity.modelInstance.getRenderables(renderables, pool);
               }
            });

            baseUI.getImGuiPanelManager().addPanel("libGDX Bullet Physics", this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            if (simulate.get())
               world.update();

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
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXLibGDXBulletPhysicsDemo4();
   }  

}
