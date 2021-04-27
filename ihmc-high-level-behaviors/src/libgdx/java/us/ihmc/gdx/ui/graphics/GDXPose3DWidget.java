package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.mesh.GDXMeshBuilder;
import us.ihmc.gdx.mesh.GDXMeshDataInterpreter;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class GDXPose3DWidget implements RenderableProvider
{
   private static final Color X_AXIS_DEFAULT_COLOR = new Color(0.9f, 0.4f, 0.4f, 0.4f);
   private static final Color Y_AXIS_DEFAULT_COLOR = new Color(0.4f, 0.9f, 0.4f, 0.4f);
   private static final Color Z_AXIS_DEFAULT_COLOR = new Color(0.4f, 0.4f, 0.9f, 0.4f);
   private static final Color CENTER_DEFAULT_COLOR = new Color(0.7f, 0.7f, 0.7f, 0.4f);

   private static final Color X_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.9f, 0.3f, 0.3f, 0.9f);
   private static final Color Y_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.3f, 0.9f, 0.3f, 0.9f);
   private static final Color Z_AXIS_SELECTED_DEFAULT_COLOR = new Color(0.3f, 0.3f, 0.9f, 0.9f);
   private static final Color CENTER_SELECTED_DEFAULT_COLOR = new Color(0.5f, 0.5f, 0.5f, 0.9f);

   private final double torusRadius = 0.075f;
   private final double torusTubeRadius = 0.01f;
   private final double arrowLengthRatio = 0.7;
   private final double arrowHeadBodyLengthRatio = 0.55;
   private final double arrowHeadBodyRadiusRatio = 2.0;
   private final double animationSpeed = 0.25 * Math.PI;
   private final double arrowBodyRadius = (float) torusTubeRadius;
   private final double arrowLength = arrowLengthRatio * torusRadius;
   private final double arrowBodyLength = (1.0 - arrowHeadBodyLengthRatio) * arrowLength;
   private final double arrowHeadRadius = arrowHeadBodyRadiusRatio * arrowBodyRadius;
   private final double arrowHeadLength = arrowHeadBodyLengthRatio * arrowLength;
   private final double arrowSpacing = 2.2 * (torusRadius + torusTubeRadius);
   private final int arrowCollisionResolution = 24;
   private final Color[] axisColors = {X_AXIS_DEFAULT_COLOR, Y_AXIS_DEFAULT_COLOR, Z_AXIS_DEFAULT_COLOR};
   private final Color[] axisSelectedColors = {X_AXIS_SELECTED_DEFAULT_COLOR, Y_AXIS_SELECTED_DEFAULT_COLOR, Z_AXIS_SELECTED_DEFAULT_COLOR};
   private final Material[] normalMaterials = new Material[3];
   private final Material[] highlightedMaterials = new Material[3];
   private final RotationMatrix[] axisRotations = new RotationMatrix[3];
   private final ModelInstance[] angularControlModelInstances = new ModelInstance[3];
   private final ModelInstance[] linearControlModelInstances = new ModelInstance[3];
   private final Torus3D angularCollisionTorus = new Torus3D();
   private final Sphere3D angularCollisionSphere = new Sphere3D();
   private final Cylinder3D arrowBaseCollisionCylinder = new Cylinder3D();
   private final Sphere3D arrowHeadCollisionSphere = new Sphere3D();
   private final Plane3D arrowHeadCollisionBaseFacingTip = new Plane3D();
   private final Plane3D arrowHeadCollisionTipFacingBase = new Plane3D();
   private final Line3D arrowHeadCollisionAxis = new Line3D();
   private final Point3D arrowHeadCollisionAxisPoint = new Point3D();
   private final ConvexPolytope3D arrowHeadCollisionCone = EuclidPolytopeFactories.newCone(arrowHeadLength, arrowHeadRadius, arrowCollisionResolution);
   private ConvexPolytope3D pickRayPolytope;
   private final Pose3D pose = new Pose3D(1.0, 0.5, 0.25, 0.0, 0.0, 0.0);
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform tempPolytopeTransform = new RigidBodyTransform();
   private GDXImGuiBasedUI baseUI;
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollider = new GilbertJohnsonKeerthiCollisionDetector();
   private final ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private final Point3D interpolatedPoint = new Point3D();
   private final Point3D adjustedRayOrigin = new Point3D();
   private final Point3D closestCollision = new Point3D();
   private EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
   private ModelInstance pickPointSphere;
   private ModelInstance arrowDebugSphere;
   private ModelInstance arrowDebugSphere2;
   private ModelInstance arrowHeadDebug;
   private SixDoFSelection closestCollisionSelection;
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private double distanceToCone;

   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
      //      Mesh angularControlHighlightMesh = angularHighlightMesh(radius, thickness);

      axisRotations[0] = new RotationMatrix(0.0, Math.PI / 2.0, 0.0);
      axisRotations[1] = new RotationMatrix(0.0, 0.0, -Math.PI / 2.0);
      axisRotations[2] = new RotationMatrix();

      for (Axis3D axis : Axis3D.values)
      {
         String axisName = axis.name().toLowerCase();

         Color color = axisColors[axis.ordinal()];
         ModelInstance arrow = GDXModelPrimitives.buildModelInstance(meshBuilder ->
         {
            // Euclid cylinders are defined from the center, but mesh builder defines them from the bottom
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing), color);
            meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing + arrowBodyLength), color);
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, -0.5 * arrowSpacing), FLIP_180, color);
            meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, -0.5 * arrowSpacing - arrowBodyLength), FLIP_180, color);
         }, axisName);
         arrow.materials.get(0).set(new BlendingAttribute(true, axisColors[axis.ordinal()].a));
         normalMaterials[axis.ordinal()] = new Material(arrow.materials.get(0));
         highlightedMaterials[axis.ordinal()] = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         highlightedMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(paletteTexture));
//         highlightedMaterials[axis.ordinal()].set(ColorAttribute.createDiffuse(Color.ORANGE));
         highlightedMaterials[axis.ordinal()].set(new BlendingAttribute(true, axisSelectedColors[axis.ordinal()].a));
         GDXTools.toGDX(axisRotations[axis.ordinal()], arrow.transform);
         linearControlModelInstances[axis.ordinal()] = arrow;
      }
      for (Axis3D axis : Axis3D.values)
      {
         String axisName = axis.name().toLowerCase();

         int resolution = 25;
         ModelInstance ring = GDXModelPrimitives.buildModelInstance(meshBuilder ->
            meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, torusRadius, torusTubeRadius, resolution, axisColors[axis.ordinal()]), axisName);
         ring.materials.get(0).set(new BlendingAttribute(true, axisColors[axis.ordinal()].a));
         GDXTools.toGDX(axisRotations[axis.ordinal()], ring.transform);
         angularControlModelInstances[axis.ordinal()] = ring;
      }

      pickPointSphere = GDXModelPrimitives.createSphere(0.005f, Color.CORAL, "pickPoint");
      double arrowSurroundingSphereRadius = arrowHeadRadius > (0.5 * arrowHeadLength) ?
            arrowHeadRadius / Math.sin(Math.atan(2.0 * arrowHeadRadius / arrowHeadLength))
            : 0.5 * arrowHeadLength;
      arrowDebugSphere2 = GDXModelPrimitives.createSphere((float) arrowSurroundingSphereRadius, Color.SLATE, "pickPointArrow");
//      arrowDebugSphere2 = GDXModelPrimitives.createSphere(0.1f, Color.SLATE, "pickPointArrow");
      arrowDebugSphere = GDXModelPrimitives.createSphere(0.001f, Color.SLATE, "pickPointArrow");
      arrowDebugSphere.materials.get(0).set(new BlendingAttribute(true, 0.2f));
      arrowDebugSphere2.materials.get(0).set(new BlendingAttribute(true, 0.2f));

      arrowHeadDebug = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, 0.0), Color.ORANGE);
      }, "arrowHeadDebug");
      arrowHeadDebug.materials.get(0).set(new BlendingAttribute(true, 0.2f));

      List<Point3D> vertices = new ArrayList<>();
      vertices.add(new Point3D(0.0, 0.0, 0.0));
      vertices.add(new Point3D(20.0, 0.0, 0.0));
      pickRayPolytope = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(vertices));
   }

   public SixDoFSelection intersect(Line3D pickRay)
   {
      return SixDoFSelection.LINEAR_X;
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (input.isWindowHovered())
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld(baseUI);

         // could do one large sphere collision to avoid completely far off picks

         closestCollisionSelection = null;
         double closestCollisionDistance = Double.POSITIVE_INFINITY;

         // collide tori
         for (Axis3D axis : Axis3D.values)
         {
            GDXTools.toEuclid(angularControlModelInstances[axis.ordinal()].transform, tempTransform);
            angularCollisionTorus.setToZero();
            angularCollisionTorus.setRadii(torusRadius, torusTubeRadius);
            angularCollisionTorus.applyTransform(tempTransform);

            angularCollisionSphere.setToZero();
            angularCollisionSphere.setRadius(torusRadius + torusTubeRadius);
            angularCollisionSphere.applyTransform(tempTransform);

            adjustedRayOrigin.setX(pickRay.getPoint().getX() - angularCollisionSphere.getPosition().getX());
            adjustedRayOrigin.setY(pickRay.getPoint().getY() - angularCollisionSphere.getPosition().getY());
            adjustedRayOrigin.setZ(pickRay.getPoint().getZ() - angularCollisionSphere.getPosition().getZ());
            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(angularCollisionSphere.getRadius(),
                                                                                                   angularCollisionSphere.getRadius(),
                                                                                                   angularCollisionSphere.getRadius(),
                                                                                                   adjustedRayOrigin,
                                                                                                   pickRay.getDirection(),
                                                                                                   firstIntersectionToPack,
                                                                                                   secondIntersectionToPack);
            if (numberOfIntersections == 2)
            {
               firstIntersectionToPack.add(angularCollisionSphere.getPosition());
               secondIntersectionToPack.add(angularCollisionSphere.getPosition());
//               boolean firstCloser = firstIntersectionToPack.distance(pickRay.getPoint()) < secondIntersectionToPack.distance(pickRay.getPoint());
               for (int i = 0; i < 100; i++)
               {
//                  if (firstCloser)
                     interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);
//                  else
//                     interpolatedPoint.interpolate(secondIntersectionToPack, firstIntersectionToPack, i / 100.0);
                  if (angularCollisionTorus.isPointInside(interpolatedPoint))
                  {
                     double distance = interpolatedPoint.distance(pickRay.getPoint());
                     if (distance < closestCollisionDistance)
                     {
                        closestCollisionDistance = distance;
                        closestCollisionSelection = SixDoFSelection.toAngularSelection(axis);
                        closestCollision.set(interpolatedPoint);
                     }
                     break;
                  }
               }
            }
         }

         // collide arrows
         for (Axis3D axis : Axis3D.values)
         {
            GDXTools.toEuclid(linearControlModelInstances[axis.ordinal()].transform, tempTransform);
            for (RobotSide side : RobotSide.values)
            {
               // collide arrow body cylinder
               {
                  arrowBaseCollisionCylinder.setToZero();
                  //               double aLittleMoreToAvoidArrow
                  arrowBaseCollisionCylinder.setSize(arrowBodyLength, arrowBodyRadius);
                  if (side == RobotSide.LEFT)
                     arrowBaseCollisionCylinder.getPosition().addZ(0.5 * arrowSpacing + 0.5 * arrowBodyLength);
                  else
                     arrowBaseCollisionCylinder.getPosition().subZ(0.5 * arrowSpacing + 0.5 * arrowBodyLength);
                  arrowBaseCollisionCylinder.applyTransform(tempTransform);

                  int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(arrowBaseCollisionCylinder.getLength(),
                                                                                                        arrowBaseCollisionCylinder.getRadius(),
                                                                                                        arrowBaseCollisionCylinder.getPosition(),
                                                                                                        arrowBaseCollisionCylinder.getAxis(),
                                                                                                        pickRay.getPoint(),
                                                                                                        pickRay.getDirection(),
                                                                                                        firstIntersectionToPack,
                                                                                                        secondIntersectionToPack);

                  if (numberOfIntersections == 2)
                  {
                     double distance = firstIntersectionToPack.distance(pickRay.getPoint());
                     if (distance < closestCollisionDistance)
                     {
                        closestCollisionDistance = distance;
                        closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
                        closestCollision.set(firstIntersectionToPack);
                     }
                  }
               }

               // collide arrow head cone
               {
                  // update cone
                  tempPolytopeTransform.setToZero();
                  arrowHeadCollisionCone.getNumberOfVertices();
                  arrowHeadCollisionCone.getVertex(0).set(0.0, 0.0, arrowHeadLength);
                  for (int i = 0; i < arrowCollisionResolution; i++)
                  {
                     double theta = i * 2.0 * Math.PI / arrowCollisionResolution;
                     double x = arrowHeadRadius * EuclidCoreTools.cos(theta);
                     double y = arrowHeadRadius * EuclidCoreTools.sin(theta);
                     arrowHeadCollisionCone.getVertex(i + 1).set(x, y, 0.0);
                  }
                  if (side == RobotSide.LEFT)
                  {
                     tempPolytopeTransform.getTranslation().addZ(0.5 * arrowSpacing + arrowBodyLength);
                  }
                  else
                  {
                     tempPolytopeTransform.getTranslation().subZ(0.5 * arrowSpacing + arrowBodyLength);
                     tempPolytopeTransform.getRotation().append(FLIP_180);

                  }
                  tempTransform.transform(tempPolytopeTransform);
                  arrowHeadCollisionCone.applyTransform(tempPolytopeTransform);
                  GDXTools.toGDX(tempPolytopeTransform, arrowHeadDebug.transform);

                  arrowHeadCollisionSphere.setToZero();
                  double arrowSurroundingSphereRadius;
                  if (arrowHeadRadius > (0.5 * arrowHeadLength))
                     arrowSurroundingSphereRadius = arrowHeadRadius / Math.sin(Math.atan(2.0 * arrowHeadRadius / arrowHeadLength));
                  else
                     arrowSurroundingSphereRadius = 0.5 * arrowHeadLength;
                  arrowHeadCollisionSphere.setRadius(arrowSurroundingSphereRadius);
//                  arrowHeadCollisionSphere.setRadius(0.1); // TODO: REMOVE
                  if (side == RobotSide.LEFT)
                     arrowHeadCollisionSphere.getPosition().addZ(0.5 * arrowSpacing + arrowBodyLength + 0.5 * arrowHeadLength);
                  else
                     arrowHeadCollisionSphere.getPosition().subZ(0.5 * arrowSpacing + arrowBodyLength + 0.5 * arrowHeadLength);
                  arrowHeadCollisionSphere.applyTransform(tempTransform);
                  GDXTools.toGDX(arrowHeadCollisionSphere.getPosition(), arrowDebugSphere2.transform);
//                  GDXTools.toGDX(arrowHeadCollisionCone.getVertex(0), arrowDebugSphere2.transform);

                  adjustedRayOrigin.setX(pickRay.getPoint().getX() - arrowHeadCollisionSphere.getPosition().getX());
                  adjustedRayOrigin.setY(pickRay.getPoint().getY() - arrowHeadCollisionSphere.getPosition().getY());
                  adjustedRayOrigin.setZ(pickRay.getPoint().getZ() - arrowHeadCollisionSphere.getPosition().getZ());
                  int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(arrowHeadCollisionSphere.getRadius(),
                                                                                                         arrowHeadCollisionSphere.getRadius(),
                                                                                                         arrowHeadCollisionSphere.getRadius(),
                                                                                                         adjustedRayOrigin,
                                                                                                         pickRay.getDirection(),
                                                                                                         firstIntersectionToPack,
                                                                                                         secondIntersectionToPack);
                  if (numberOfIntersections == 2)
                  {
                     firstIntersectionToPack.add(arrowHeadCollisionSphere.getPosition());
                     secondIntersectionToPack.add(arrowHeadCollisionSphere.getPosition());

                     arrowHeadCollisionBaseFacingTip.setToZero();
                     arrowHeadCollisionBaseFacingTip.applyTransform(tempPolytopeTransform);

                     arrowHeadCollisionTipFacingBase.setToZero();
                     arrowHeadCollisionTipFacingBase.getPoint().addZ(arrowHeadLength);
                     arrowHeadCollisionTipFacingBase.getNormal().set(0.0, 0.0, -1.0);
                     arrowHeadCollisionTipFacingBase.applyTransform(tempPolytopeTransform);

                     arrowHeadCollisionAxis.set(arrowHeadCollisionBaseFacingTip.getPoint(), arrowHeadCollisionBaseFacingTip.getNormal());

                     boolean firstCloser = firstIntersectionToPack.distance(pickRay.getPoint()) < secondIntersectionToPack.distance(pickRay.getPoint());
                     for (int i = 0; i < 100; i++)
                     {
                        if (firstCloser)
                           interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);
                        else
                           interpolatedPoint.interpolate(secondIntersectionToPack, firstIntersectionToPack, i / 100.0);

                        if (arrowHeadCollisionBaseFacingTip.isOnOrAbove(interpolatedPoint) && arrowHeadCollisionTipFacingBase.isOnOrAbove(interpolatedPoint))
                        {
                           arrowHeadCollisionAxis.orthogonalProjection(interpolatedPoint, arrowHeadCollisionAxisPoint);
                           double distanceFromBase = arrowHeadCollisionAxisPoint.distance(arrowHeadCollisionBaseFacingTip.getPoint());
                           double radiusBoundsAtTier = EuclidCoreTools.interpolate(arrowHeadRadius, 0.0, distanceFromBase / arrowHeadLength);
                           if (arrowHeadCollisionAxisPoint.distance(interpolatedPoint) <= radiusBoundsAtTier)
                           {
                              double distance = interpolatedPoint.distance(pickRay.getPoint());
                              if (distance < closestCollisionDistance)
                              {
                                 closestCollisionDistance = distance;
                                 closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
                                 closestCollision.set(interpolatedPoint);
                              }
                              break;
                           }
                        }


                        GDXTools.toGDX(interpolatedPoint, arrowDebugSphere.transform);
//                        distanceToCone = arrowHeadCollisionCone.signedDistance(interpolatedPoint);
////                        if (arrowHeadCollisionCone.isPointInside(interpolatedPoint))
//                        if (distanceToCone < 0.005)
//                        {
//                           double distance = interpolatedPoint.distance(pickRay.getPoint());
//                           if (distance < closestCollisionDistance)
//                           {
//                              closestCollisionDistance = distance;
//                              closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
//                              closestCollision.set(interpolatedPoint);
//                           }
//                           break;
//                        }
                     }
                  }
               }
//               pickRayPolytope.getVertex(0).set(pickRay.getPoint());
//               pickRayPolytope.getVertex(1).set(pickRay.getDirection());
//               pickRayPolytope.getVertex(1).scale(20.0);
//               pickRayPolytope.getVertex(1).add(pickRay.getPoint());
//
//
//               expandingPolytopeAlgorithm.evaluateCollision(pickRayPolytope, arrowHeadCollisionCone, collisionResult);
//
//               GDXTools.toGDX(collisionResult.getPointOnA(), arrowDebugSphere.transform);
//               GDXTools.toGDX(collisionResult.getPointOnA(), arrowDebugSphere2.transform);
//
//               if (collisionResult.areShapesColliding())
//               {
////                  collisionResult.getPointOnA()
//                  LogTools.info("Colliding");
//
//               }
            }
         }

         // could only do this when selection changed
         for (Axis3D axis : Axis3D.values)
         {
            if (closestCollisionSelection != null && closestCollisionSelection.isAngular() && closestCollisionSelection.toAxis3D() == axis)
            {
               angularControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(highlightedMaterials[axis.ordinal()]);
            }
            else
            {
               angularControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(normalMaterials[axis.ordinal()]);
            }

            if (closestCollisionSelection != null && closestCollisionSelection.isLinear() && closestCollisionSelection.toAxis3D() == axis)
            {
               linearControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(highlightedMaterials[axis.ordinal()]);
            }
            else
            {
               linearControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(normalMaterials[axis.ordinal()]);
            }
         }

         GDXTools.toGDX(closestCollision, pickPointSphere.transform);
      }
   }

   public void render()
   {
      for (Axis3D axis : Axis3D.values)
      {
         tempTransform.set(pose.getOrientation(), pose.getPosition());
         tempTransform.appendOrientation(axisRotations[axis.ordinal()]);
         GDXTools.toGDX(tempTransform, linearControlModelInstances[axis.ordinal()].transform);
         GDXTools.toGDX(tempTransform, angularControlModelInstances[axis.ordinal()].transform);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (Axis3D axis : Axis3D.values)
      {
         linearControlModelInstances[axis.ordinal()].getRenderables(renderables, pool);
         angularControlModelInstances[axis.ordinal()].getRenderables(renderables, pool);
      }

      if (closestCollisionSelection != null)
         pickPointSphere.getRenderables(renderables, pool);

//      arrowDebugSphere.getRenderables(renderables, pool);
//      arrowDebugSphere2.getRenderables(renderables, pool);
//      arrowHeadDebug.getRenderables(renderables, pool);
   }

   public static Mesh angularHighlightMesh(double majorRadius, double minorRadius)
   {
      return tetrahedronRingMesh(1.75 * minorRadius, 1.25 * minorRadius, 5);
   }

   public static Mesh linearControlMesh(double bodyRadius, double bodyLength, double headRadius, double headLength, double spacing)
   {
      GDXMeshBuilder meshBuilder = new GDXMeshBuilder();
      meshBuilder.addCylinder(bodyLength, bodyRadius, new Point3D(0.0, 0.0, 0.5 * spacing));
      meshBuilder.addCone(headLength, headRadius, new Point3D(0.0, 0.0, 0.5 * spacing + bodyLength));
      meshBuilder.addCylinder(bodyLength, bodyRadius, new Point3D(0.0, 0.0, -0.5 * spacing), new YawPitchRoll(0.0, Math.PI, 0.0));
      meshBuilder.addCone(headLength, headRadius, new Point3D(0.0, 0.0, -0.5 * spacing - bodyLength), new YawPitchRoll(0.0, Math.PI, 0.0));
      return meshBuilder.generateMesh();
   }

   public static Mesh linearControlHighlightMesh(double bodyRadius, double bodyLength, double spacing)
   {
      GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

      int numberOfHighlights = 5;

      Point3D center = new Point3D(0, 0, 0.5 * spacing + 0.33 * bodyLength);
      MeshDataHolder ringMesh = tetrahedronRingMeshDataHolder(1.75 * bodyRadius, 1.25 * bodyRadius, numberOfHighlights);
      meshBuilder.addMesh(ringMesh, center);
      center.negate();
      meshBuilder.addMesh(ringMesh, center);

      return meshBuilder.generateMesh();
   }

   public static Mesh tetrahedronRingMesh(double ringRadius, double tetrahedronSize, int numberOfTetrahedrons)
   {
      return GDXMeshDataInterpreter.interpretMeshData(tetrahedronRingMeshDataHolder(ringRadius, tetrahedronSize, numberOfTetrahedrons));
   }

   public static MeshDataHolder tetrahedronRingMeshDataHolder(double ringRadius, double tetrahedronSize, int numberOfTetrahedrons)
   {
      GDXMeshBuilder meshBuilder = new GDXMeshBuilder();

      Point3D position = new Point3D();
      Point3D offset = new Point3D();
      Quaternion orientation = new Quaternion();

      for (int i = 0; i < numberOfTetrahedrons; i++)
      {
         MeshDataHolder tetrahedron = MeshDataGenerator.Tetrahedron(tetrahedronSize);
         orientation.setToYawOrientation(i * 2.0 * Math.PI / numberOfTetrahedrons);
         orientation.appendPitchRotation(0.5 * Math.PI);

         offset.set(0.0, 0.0, ringRadius);
         orientation.transform(offset);
         position.set(offset);
         meshBuilder.addMesh(tetrahedron, position, orientation);
      }

      return meshBuilder.generateMeshDataHolder();
   }
}
