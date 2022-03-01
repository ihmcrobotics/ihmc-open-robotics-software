package us.ihmc.gdx.simulation.bullet.libgdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.ContactResultCallback;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObjectWrapper;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.simulation.bullet.GDXBulletTools;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLabFloorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.yo.ImPlotYoPlot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class GDXBulletPhysicsInteractionForcesDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXEnvironmentBuilder environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
   private final ImFloat blockTransparency = new ImFloat(0.2f);

   public GDXBulletPhysicsInteractionForcesDemo()
   {
      GDXBulletTools.ensureBulletInitialized();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private final ArrayList<btManifoldPoint> contactPoints = new ArrayList<>();
         private final RecyclingArrayList<ModelInstance> pointsOnA = new RecyclingArrayList<>(this::createSphere);
         private final RecyclingArrayList<ModelInstance> arrowsOnB = new RecyclingArrayList<>(this::createArrow);

         private GDXLabFloorObject labFloorObject;
         private GDXMediumCinderBlockRoughed mediumCinderBlockRoughed;
         private final YoRegistry yoRegistry = new YoRegistry("InteractionForcesDemo");
         private final YoInteger numberOfContactPoints = new YoInteger("numberOfContactPoints", yoRegistry);
         private final ImPlotYoPlot numberOfContactPointsPlot = new ImPlotYoPlot(numberOfContactPoints);
         private final YoDouble pointAX = new YoDouble("pointAX", yoRegistry);
         private final YoDouble pointAY = new YoDouble("pointAY", yoRegistry);
         private final YoDouble pointAZ = new YoDouble("pointAZ", yoRegistry);
         private final ImPlotYoPlot pointAPlot = new ImPlotYoPlot(pointAX, pointAY, pointAZ);
         private final YoDouble pointBX = new YoDouble("pointBX", yoRegistry);
         private final YoDouble pointBY = new YoDouble("pointBY", yoRegistry);
         private final YoDouble pointBZ = new YoDouble("pointBZ", yoRegistry);
         private final ImPlotYoPlot pointBPlot = new ImPlotYoPlot(pointBX, pointBY, pointBZ);
         private final YoDouble normalX = new YoDouble("normalX", yoRegistry);
         private final YoDouble normalY = new YoDouble("normalY", yoRegistry);
         private final YoDouble normalZ = new YoDouble("normalZ", yoRegistry);
         private final ImPlotYoPlot normalPlot = new ImPlotYoPlot(normalX, normalY, normalZ);

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.get3DSceneManager().addCoordinateFrame(0.3);

            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            labFloorObject = new GDXLabFloorObject();
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            baseUI.get3DSceneManager().addRenderableProvider(this::getRenderables);

            recreateAndPlace();

            environmentBuilder.getBulletPhysicsManager().addPostTickRunnable(() ->
            {
               ContactResultCallback contactResultCallback = new ContactResultCallback()
               {
                  @Override
                  public float addSingleResult(btManifoldPoint contactPoint,
                                               btCollisionObjectWrapper collisionObjectAWrapper,
                                               int partIdA,
                                               int indexA,
                                               btCollisionObjectWrapper collisionObjectBWrapper,
                                               int partIdB,
                                               int indexB)
                  {
//                     float superResult = super.addSingleResult(contactPoint,
//                                                               collisionObjectAWrapper,
//                                                               partIdA,
//                                                               indexA,
//                                                               collisionObjectBWrapper,
//                                                               partIdB,
//                                                               indexB);

                     contactPoints.add(contactPoint);

//                     LogTools.info(contactPoint);
//                     LogTools.info("partIdA: {}  indexA {}", partIdA, indexA);
//                     LogTools.info("partIdB: {}  indexB {}", partIdA, indexA);

                     return 0;
                  }
               };
               contactPoints.clear();
               environmentBuilder.getBulletPhysicsManager()
                                 .getMultiBodyDynamicsWorld()
                                 .contactPairTest(labFloorObject.getBtRigidBody(), mediumCinderBlockRoughed.getBtRigidBody(), contactResultCallback);
//               LogTools.info("Number of contact points: {}", contactPoints.size());
               numberOfContactPoints.set(contactPoints.size());
               pointsOnA.clear();
               arrowsOnB.clear();
               for (btManifoldPoint contactPoint : contactPoints)
               {
                  Vector3 pointOnA = new Vector3();
                  contactPoint.getPositionWorldOnA(pointOnA);
                  pointsOnA.add().transform.setToTranslation(pointOnA);
                  pointAX.set(pointOnA.x);
                  pointAY.set(pointOnA.y);
                  pointAZ.set(pointOnA.z);

                  RigidBodyTransform arrowTransform = new RigidBodyTransform();
                  Vector3 normalOnBGDX = new Vector3();
                  contactPoint.getNormalWorldOnB(normalOnBGDX);
                  normalX.set(normalOnBGDX.x);
                  normalY.set(normalOnBGDX.y);
                  normalZ.set(normalOnBGDX.z);
                  Vector3D normalOnBEuclid = new Vector3D();
                  GDXTools.toEuclid(normalOnBGDX, normalOnBEuclid);
                  normalOnBEuclid.normalize();
                  normalX.set(normalOnBEuclid.getX());
                  normalY.set(normalOnBEuclid.getY());
                  normalZ.set(normalOnBEuclid.getZ());
                  Quaternion arrowOrientation = new Quaternion();
                  EuclidGeometryTools.orientation3DFromZUpToVector3D(normalOnBEuclid, arrowOrientation);
                  Vector3 pointOnB = new Vector3();
                  contactPoint.getPositionWorldOnB(pointOnB);
                  pointBX.set(pointOnB.x);
                  pointBY.set(pointOnB.y);
                  pointBZ.set(pointOnB.z);
                  Point3D pointOnBEuclid = new Point3D();
                  GDXTools.toEuclid(pointOnB, pointOnBEuclid);
                  arrowTransform.set(arrowOrientation, pointOnBEuclid);
                  ModelInstance arrow = arrowsOnB.add();
                  GDXTools.toGDX(arrowTransform, arrow.transform);
//                  arrow.transform.setToRotationRad(normalOnB, 0.0f);
//                  arrow.transform.translate(pointOnB);
               }
            });

            ImGuiPanel experimentPanel = new ImGuiPanel("Demo", () ->
            {
               if (ImGui.button("Replace Block"))
               {
                  recreateAndPlace();
               }
               ImGui.sliderFloat("Block transparency", blockTransparency.getData(), 0.0f, 1.0f);
               GDXTools.setTransparency(mediumCinderBlockRoughed.getRealisticModelInstance(), blockTransparency.get());
               numberOfContactPointsPlot.render(environmentBuilder.getBulletPhysicsManager().getSimulate().get());
               pointAPlot.render(environmentBuilder.getBulletPhysicsManager().getSimulate().get());
               pointBPlot.render(environmentBuilder.getBulletPhysicsManager().getSimulate().get());
               normalPlot.render(environmentBuilder.getBulletPhysicsManager().getSimulate().get());
            });
            baseUI.getImGuiPanelManager().addPanel(experimentPanel);

            baseUI.get3DSceneManager().getCamera3D().changeCameraPosition(2.0, 1.0, 1.0);
            environmentBuilder.getBulletPhysicsManager().getSimulationRate().set(0.1f);
         }

         public void recreateAndPlace()
         {
            if (mediumCinderBlockRoughed != null)
            {
               environmentBuilder.removeObject(mediumCinderBlockRoughed);
            }

            mediumCinderBlockRoughed = new GDXMediumCinderBlockRoughed();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.5);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(45.0), Math.toRadians(45.0), Math.toRadians(15.0));
            mediumCinderBlockRoughed.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(mediumCinderBlockRoughed);
            mediumCinderBlockRoughed.copyThisTransformToBulletMultiBody();
         }

         private ModelInstance createSphere()
         {
            ModelInstance modelInstance = GDXModelPrimitives.createSphere(0.01f, Color.RED);
//            GDXTools.setTransparency(modelInstance, 0.8f);
            return modelInstance;
         }

         private ModelInstance createArrow()
         {
//            ModelInstance modelInstance = GDXModelPrimitives.createSphere(0.01f, Color.PINK);
            float length = 0.2f;
            Color color = Color.PINK;
            ModelInstance modelInstance = GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               double coneHeight = 0.10 * length;
               double cylinderLength = length - coneHeight;
               double cylinderRadius = cylinderLength / 20.0;
               double coneRadius = 1.5 * cylinderRadius;
               meshBuilder.addCylinder(cylinderLength, cylinderRadius, new Point3D(), color);
               meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, cylinderLength), color);
            });
//            GDXTools.setTransparency(modelInstance, 0.8f);
            return modelInstance;
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            for (ModelInstance modelInstance : pointsOnA)
            {
               modelInstance.getRenderables(renderables, pool);
            }
            for (ModelInstance modelInstance : arrowsOnB)
            {
               modelInstance.getRenderables(renderables, pool);
            }
         }

         @Override
         public void dispose()
         {
            environmentBuilder.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXBulletPhysicsInteractionForcesDemo();
   }
}
