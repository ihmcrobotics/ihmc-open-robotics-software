package us.ihmc.rdx.simulation.bullet.libgdx;

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
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.bullet.RDXBulletTools;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.objects.RDXLabFloorObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXMediumCinderBlockRoughed;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.yo.ImPlotYoPlot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

/**
 * https://web.archive.org/web/20170706235814/http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Main_Page
 * https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=2568
 * https://github.com/kripken/bullet/blob/master/Demos/CollisionInterfaceDemo/CollisionInterfaceDemo.cpp
 */
public class RDXBulletPhysicsInteractionForcesDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXEnvironmentBuilder environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
   private final ImFloat blockTransparency = new ImFloat(0.2f);

   public RDXBulletPhysicsInteractionForcesDemo()
   {
      RDXBulletTools.ensureBulletInitialized();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private final ArrayList<btManifoldPoint> contactPoints = new ArrayList<>();
         private final RecyclingArrayList<ModelInstance> pointsOnA = new RecyclingArrayList<>(this::createSphere);
         private final RecyclingArrayList<ModelInstance> arrowsOnB = new RecyclingArrayList<>(this::createArrow);

         private RDXLabFloorObject labFloorObject;
         private RDXMediumCinderBlockRoughed fallingBlock;
         private RDXMediumCinderBlockRoughed sittingBlock;
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
         private final YoDouble distance = new YoDouble("distance", yoRegistry);
         private final YoDouble appliedImpulse = new YoDouble("appliedImpulse", yoRegistry);
         private final YoDouble appliedImpulseLateral1 = new YoDouble("appliedImpulseLateral1", yoRegistry);
         private final YoDouble appliedImpulseLateral2 = new YoDouble("appliedImpulseLateral2", yoRegistry);
         private final YoDouble combinedContactDamping1 = new YoDouble("combinedContactDamping1", yoRegistry);
         private final YoDouble combinedFriction = new YoDouble("combinedFriction", yoRegistry);
         private final YoDouble combinedRestitution = new YoDouble("combinedRestitution", yoRegistry);
         private final YoDouble combinedRollingFriction = new YoDouble("combinedRollingFriction", yoRegistry);
         private final YoDouble combinedSpinningFriction = new YoDouble("combinedSpinningFriction", yoRegistry);
         private final YoDouble contactCFM = new YoDouble("contactCFM", yoRegistry);
         private final YoDouble contactERP = new YoDouble("contactERP", yoRegistry);
         private final YoDouble contactMotion1 = new YoDouble("contactMotion1", yoRegistry);
         private final YoDouble contactMotion2 = new YoDouble("contactMotion2", yoRegistry);
         private final YoInteger contactPointFlags = new YoInteger("contactPointFlags", yoRegistry);
         private final YoDouble distance1 = new YoDouble("distance1", yoRegistry);
         private final YoDouble frictionCFM = new YoDouble("frictionCFM", yoRegistry);
         private final YoDouble lateralFrictionDirection1X = new YoDouble("lateralFrictionDirection1X", yoRegistry);
         private final YoDouble lateralFrictionDirection1Y = new YoDouble("lateralFrictionDirection1Y", yoRegistry);
         private final YoDouble lateralFrictionDirection1Z = new YoDouble("lateralFrictionDirection1Z", yoRegistry);
         private final YoDouble lateralFrictionDirection2X = new YoDouble("lateralFrictionDirection2X", yoRegistry);
         private final YoDouble lateralFrictionDirection2Y = new YoDouble("lateralFrictionDirection2Y", yoRegistry);
         private final YoDouble lateralFrictionDirection2Z = new YoDouble("lateralFrictionDirection2Z", yoRegistry);
         private final YoInteger lifeTime = new YoInteger("lifeTime", yoRegistry);
         private final ImPlotYoPlot distancePlot = new ImPlotYoPlot(distance);
         private final ImPlotYoPlot appliedImpulsePlot = new ImPlotYoPlot(appliedImpulse);
         private final ImPlotYoPlot appliedImpulseLateral1Plot = new ImPlotYoPlot(appliedImpulseLateral1);
         private final ImPlotYoPlot appliedImpulseLateral2Plot = new ImPlotYoPlot(appliedImpulseLateral2);
         private final ImPlotYoPlot combinedContactDamping1Plot = new ImPlotYoPlot(combinedContactDamping1);
         private final ImPlotYoPlot combinedFrictionPlot = new ImPlotYoPlot(combinedFriction);
         private final ImPlotYoPlot combinedRestitutionPlot = new ImPlotYoPlot(combinedRestitution);
         private final ImPlotYoPlot combinedRollingFrictionPlot = new ImPlotYoPlot(combinedRollingFriction);
         private final ImPlotYoPlot combinedSpinningFrictionPlot = new ImPlotYoPlot(combinedSpinningFriction);
         private final ImPlotYoPlot contactCFMPlot = new ImPlotYoPlot(contactCFM);
         private final ImPlotYoPlot contactERPPlot = new ImPlotYoPlot(contactERP);
         private final ImPlotYoPlot contactMotion1Plot = new ImPlotYoPlot(contactMotion1);
         private final ImPlotYoPlot contactMotion2Plot = new ImPlotYoPlot(contactMotion2);
         private final ImPlotYoPlot contactPointFlagsPlot = new ImPlotYoPlot(contactPointFlags);
         private final ImPlotYoPlot distance1Plot = new ImPlotYoPlot(distance1);
         private final ImPlotYoPlot frictionCFMPlot = new ImPlotYoPlot(frictionCFM);
         private final ImPlotYoPlot lateralFrictionDirection1Plot = new ImPlotYoPlot(lateralFrictionDirection1X, lateralFrictionDirection1Y, lateralFrictionDirection1Z);
         private final ImPlotYoPlot lateralFrictionDirection2Plot = new ImPlotYoPlot(lateralFrictionDirection2X, lateralFrictionDirection2Y, lateralFrictionDirection2Z);
         private final ImPlotYoPlot lifeTimePlot = new ImPlotYoPlot(lifeTime);

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            labFloorObject = new RDXLabFloorObject();
            environmentBuilder.addObject(labFloorObject);
            labFloorObject.copyThisTransformToBulletMultiBody();

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

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
//               environmentBuilder.getBulletPhysicsManager()
//                                 .getMultiBodyDynamicsWorld()
//                                 .contactPairTest(labFloorObject.getBtRigidBody(), mediumCinderBlockRoughed.getBtRigidBody(), contactResultCallback);
//               environmentBuilder.getBulletPhysicsManager()
//                                 .getMultiBodyDynamicsWorld()
//                                 .contactPairTest(fallingBlock.getBtRigidBody(), labFloorObject.getBtRigidBody(), contactResultCallback);
               environmentBuilder.getBulletPhysicsManager()
                                 .getMultiBodyDynamicsWorld()
                                 .contactPairTest(fallingBlock.getBtRigidBody(), sittingBlock.getBtRigidBody(), contactResultCallback);
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
                  LibGDXTools.toEuclid(normalOnBGDX, normalOnBEuclid);
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
                  LibGDXTools.toEuclid(pointOnB, pointOnBEuclid);
                  arrowTransform.set(arrowOrientation, pointOnBEuclid);
                  ModelInstance arrow = arrowsOnB.add();
                  LibGDXTools.toLibGDX(arrowTransform, arrow.transform);
//                  arrow.transform.setToRotationRad(normalOnB, 0.0f);
//                  arrow.transform.translate(pointOnB);

                  float distance = contactPoint.getDistance();
                  float appliedImpulse = contactPoint.getAppliedImpulse();
                  float appliedImpulseLateral1 = contactPoint.getAppliedImpulseLateral1();
                  float appliedImpulseLateral2 = contactPoint.getAppliedImpulseLateral2();
                  float combinedContactDamping1 = contactPoint.getCombinedContactDamping1();
                  float combinedFriction = contactPoint.getCombinedFriction();
                  float combinedRestitution = contactPoint.getCombinedRestitution();
                  float combinedRollingFriction = contactPoint.getCombinedRollingFriction();
                  float combinedSpinningFriction = contactPoint.getCombinedSpinningFriction();
                  float contactCFM = contactPoint.getContactCFM();
                  float contactERP = contactPoint.getContactERP();
                  float contactMotion1 = contactPoint.getContactMotion1();
                  float contactMotion2 = contactPoint.getContactMotion2();
                  int contactPointFlags = contactPoint.getContactPointFlags();
                  float distance1 = contactPoint.getDistance1();
                  float frictionCFM = contactPoint.getFrictionCFM();
                  Vector3 lateralFrictionDirection1 = new Vector3();
                  contactPoint.getLateralFrictionDir1(lateralFrictionDirection1);
                  Vector3 lateralFrictionDirection2 = new Vector3();
                  contactPoint.getLateralFrictionDir2(lateralFrictionDirection2);
                  int lifeTime = contactPoint.getLifeTime();

                  this.distance.set(distance);
                  this.appliedImpulse.set(appliedImpulse);
                  this.appliedImpulseLateral1.set(appliedImpulseLateral1);
                  this.appliedImpulseLateral2.set(appliedImpulseLateral2);
                  this.combinedContactDamping1.set(combinedContactDamping1);
                  this.combinedFriction.set(combinedFriction);
                  this.combinedRestitution.set(combinedRestitution);
                  this.combinedRollingFriction.set(combinedRollingFriction);
                  this.combinedSpinningFriction.set(combinedSpinningFriction);
                  this.contactCFM.set(contactCFM);
                  this.contactERP.set(contactERP);
                  this.contactMotion1.set(contactMotion1);
                  this.contactMotion2.set(contactMotion2);
                  this.contactPointFlags.set(contactPointFlags);
                  this.distance1.set(distance1);
                  this.frictionCFM.set(frictionCFM);
                  this.lateralFrictionDirection1X.set(lateralFrictionDirection1.x);
                  this.lateralFrictionDirection1Y.set(lateralFrictionDirection1.y);
                  this.lateralFrictionDirection1Z.set(lateralFrictionDirection1.z);
                  this.lateralFrictionDirection2X.set(lateralFrictionDirection2.x);
                  this.lateralFrictionDirection2Y.set(lateralFrictionDirection2.y);
                  this.lateralFrictionDirection2Z.set(lateralFrictionDirection2.z);
                  this.lifeTime.set(lifeTime);
               }
            });

            RDXPanel experimentPanel = new RDXPanel("Demo", () ->
            {
               if (ImGui.button("Replace Block"))
               {
                  recreateAndPlace();
               }
               ImGui.sliderFloat("Block transparency", blockTransparency.getData(), 0.0f, 1.0f);
               LibGDXTools.setOpacity(fallingBlock.getRealisticModelInstance(), blockTransparency.get());
            });
            baseUI.getImGuiPanelManager().addPanel(experimentPanel);

            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(2.0, 1.0, 1.0);
         }

         public void recreateAndPlace()
         {
            if (fallingBlock != null)
            {
               environmentBuilder.removeObject(fallingBlock);
               environmentBuilder.removeObject(sittingBlock);
            }

            fallingBlock = new RDXMediumCinderBlockRoughed();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.5);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(45.0), Math.toRadians(45.0), Math.toRadians(15.0));
            fallingBlock.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(fallingBlock);
            fallingBlock.copyThisTransformToBulletMultiBody();

            sittingBlock = new RDXMediumCinderBlockRoughed();
            transformToWorld = new RigidBodyTransform();
            transformToWorld.getTranslation().set(0.0, 0.0, 0.1);
            transformToWorld.getRotation().setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
            sittingBlock.setTransformToWorld(transformToWorld);
            environmentBuilder.addObject(sittingBlock);
            sittingBlock.copyThisTransformToBulletMultiBody();
         }

         private ModelInstance createSphere()
         {
            ModelInstance modelInstance = RDXModelBuilder.createSphere(0.01f, Color.RED);
//            LibGDXTools.setTransparency(modelInstance, 0.8f);
            return modelInstance;
         }

         private ModelInstance createArrow()
         {
//            ModelInstance modelInstance = RDXModelBuilder.createSphere(0.01f, Color.PINK);
            float length = 0.2f;
            Color color = Color.PINK;
            ModelInstance modelInstance = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               double coneHeight = 0.10 * length;
               double cylinderLength = length - coneHeight;
               double cylinderRadius = cylinderLength / 20.0;
               double coneRadius = 1.5 * cylinderRadius;
               meshBuilder.addCylinder(cylinderLength, cylinderRadius, new Point3D(), color);
               meshBuilder.addCone(coneHeight, coneRadius, new Point3D(0.0, 0.0, cylinderLength), color);
            });
//            LibGDXTools.setTransparency(modelInstance, 0.8f);
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
      new RDXBulletPhysicsInteractionForcesDemo();
   }
}
