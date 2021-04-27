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
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
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

   private final double radius = 0.075f;
   private final double thickness = 0.01f;
   private final double arrowLengthRatio = 0.7;
   private final double arrowHeadBodyLengthRatio = 0.55;
   private final double arrowHeadBodyRadiusRatio = 2.0;
   private final double animationSpeed = 0.25 * Math.PI;
   private final double bodyRadius = (float) thickness;
   private final double arrowLength = arrowLengthRatio * radius;
   private final double bodyLength = (1.0 - arrowHeadBodyLengthRatio) * arrowLength;
   private final double headRadius = arrowHeadBodyRadiusRatio * bodyRadius;
   private final double headLength = arrowHeadBodyLengthRatio * arrowLength;
   private final double spacing = 2.2 * (radius + thickness);
   private final Color[] axisColors = {X_AXIS_DEFAULT_COLOR, Y_AXIS_DEFAULT_COLOR, Z_AXIS_DEFAULT_COLOR};
   private final Color[] axisSelectedColors = {X_AXIS_SELECTED_DEFAULT_COLOR, Y_AXIS_SELECTED_DEFAULT_COLOR, Z_AXIS_SELECTED_DEFAULT_COLOR};
   private final Material[] normalMaterials = new Material[3];
   private final Material[] highlightedMaterials = new Material[3];
   private final RotationMatrix[] axisRotations = new RotationMatrix[3];
   private final ModelInstance[] angularControlModelInstances = new ModelInstance[3];
   private final ModelInstance[] linearControlModelInstances = new ModelInstance[3];
   private final Torus3D angularCollisionTorus = new Torus3D();
   private final Sphere3D angularCollisionSphere = new Sphere3D();
   private final Cylinder3D pickCylinder = new Cylinder3D();


   private final Pose3D pose = new Pose3D(1.0, 0.5, 0.25, 0.0, 0.0, 0.0);
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private GDXImGuiBasedUI baseUI;
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollider = new GilbertJohnsonKeerthiCollisionDetector();
   private final ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private final Point3D interpolatedPoint = new Point3D();
   private final Point3D adjustedRayOrigin = new Point3D();
   private EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

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
            meshBuilder.addCylinder(bodyLength, bodyRadius, new Point3D(0.0, 0.0, 0.5 * spacing), color);
            meshBuilder.addCone(headLength, headRadius, new Point3D(0.0, 0.0, 0.5 * spacing + bodyLength), color);
            meshBuilder.addCylinder(bodyLength, bodyRadius, new Point3D(0.0, 0.0, -0.5 * spacing), new YawPitchRoll(0.0, Math.PI, 0.0), color);
            meshBuilder.addCone(headLength, headRadius, new Point3D(0.0, 0.0, -0.5 * spacing - bodyLength), new YawPitchRoll(0.0, Math.PI, 0.0), color);
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
            meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, radius, thickness, resolution, axisColors[axis.ordinal()]), axisName);
         ring.materials.get(0).set(new BlendingAttribute(true, axisColors[axis.ordinal()].a));
         GDXTools.toGDX(axisRotations[axis.ordinal()], ring.transform);
         angularControlModelInstances[axis.ordinal()] = ring;
      }
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

//         baseUI.getSceneManager().getCamera3D().frustum.

         for (Axis3D axis : Axis3D.values)
         {
//            GDXTools.toGDX(tempTransform, angularControlModelInstances[axis.ordinal()].transform);
            GDXTools.toEuclid(angularControlModelInstances[axis.ordinal()].transform, tempTransform);
            angularCollisionTorus.setToZero();
            angularCollisionTorus.setRadii(radius, thickness);
            angularCollisionTorus.applyTransform(tempTransform);

            angularCollisionSphere.setToZero();
            angularCollisionSphere.setRadius(radius + thickness);
            angularCollisionSphere.applyTransform(tempTransform);

//            angularCollisionSphere.intersectionWith()


//            EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D()
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
            if (numberOfIntersections >= 1)
               firstIntersectionToPack.add(angularCollisionSphere.getPosition());
            if (numberOfIntersections == 2)
               secondIntersectionToPack.add(angularCollisionSphere.getPosition());

            boolean colliding = false;
            if (numberOfIntersections == 1)
            {
               colliding = angularCollisionTorus.isPointInside(firstIntersectionToPack);
            }
            else if (numberOfIntersections == 2)
            {
               for (int i = 0; i < 100; i++)
               {
                  interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);
                  if (angularCollisionTorus.isPointInside(interpolatedPoint))
                  {
                     colliding = true;
                     break;
                  }
               }
            }

//            angularCollisionTorus.distance()

//            pickCylinder.set(pickRay.getPoint(), pickRay.getDirection(), 20.0, 0.001);

//            gjkCollider.evaluateCollision(angularCollisionTorus, pickCylinder, collisionResult);
//            expandingPolytopeAlgorithm.evaluateCollision(angularCollisionTorus, pickCylinder, collisionResult);

            if (colliding)
            {
               angularControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(highlightedMaterials[axis.ordinal()]);
            }
            else
            {
               angularControlModelInstances[axis.ordinal()].nodes.get(0).parts.get(0).material.set(normalMaterials[axis.ordinal()]);
            }
         }
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
