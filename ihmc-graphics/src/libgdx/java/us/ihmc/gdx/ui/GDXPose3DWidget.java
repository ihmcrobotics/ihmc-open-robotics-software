package us.ihmc.gdx.ui;

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
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.mesh.GDXMeshBuilder;
import us.ihmc.gdx.mesh.GDXMeshDataInterpreter;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.robotics.robotSide.RobotSide;

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

   private final ImFloat torusRadius = new ImFloat(0.5f);
   private final ImFloat torusTubeRadius = new ImFloat(0.016f);
   private final ImFloat arrowLengthRatio = new ImFloat(0.3f);
   private final ImFloat arrowHeadBodyLengthRatio = new ImFloat(0.3f);
   private final ImFloat arrowHeadBodyRadiusRatio = new ImFloat(2.0f);
   private final ImFloat arrowSpacingFactor = new ImFloat(2.2f);
   private double animationSpeed = 0.25 * Math.PI;
   private double arrowBodyRadius;
   private double arrowLength;
   private double arrowBodyLength;
   private double arrowHeadRadius;
   private double arrowHeadLength;
   private double arrowSpacing;
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
   private final Pose3D pose = new Pose3D();
   /** The main, source, true, base transform that this thing represents. */
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform tempPolytopeTransform = new RigidBodyTransform();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private final Point3D interpolatedPoint = new Point3D();
   private final Point3D adjustedRayOrigin = new Point3D();
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection;
   private static final YawPitchRoll FLIP_180 = new YawPitchRoll(0.0, Math.PI, 0.0);
   private boolean dragging = false;
   private float mouseDraggedX = 0.0f;
   private float mouseDraggedY = 0.0f;
   private float dragBucketX;
   private float dragBucketY;
   private final Line3D axisDragLine = new Line3D();
   private final Plane3D axisDragPlane = new Plane3D();
   private final Point3D axisDragLineClosest = new Point3D();
   private final Point3D axisCollisionWithAngularPickPlane = new Point3D();
   private final Point3D angularDragPlaneIntersection = new Point3D();
   private final Point3D angularDragPlaneIntersectionPrevious = new Point3D();
   private final Vector3D clockHandVector = new Vector3D();
   private final Vector3D previousClockHandVector = new Vector3D();
   private final Vector3D crossProduct = new Vector3D();
   private final Vector3D axisMoveVector = new Vector3D();
   private final AxisAngle axisAngleToRotateBy = new AxisAngle();
   private final String imGuiWindowName;

   public GDXPose3DWidget(String name)
   {
      imGuiWindowName = ImGuiTools.uniqueLabel("3D Widget (" + name + ")");
   }

   public void create()
   {
      axisRotations[0] = new RotationMatrix(0.0, Math.PI / 2.0, 0.0);
      axisRotations[1] = new RotationMatrix(0.0, 0.0, -Math.PI / 2.0);
      axisRotations[2] = new RotationMatrix();

      recreateGraphics();
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      boolean rightMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Right);
      float mouseDragDeltaX = ImGui.getMouseDragDeltaX(ImGuiMouseButton.Right);
      float mouseDragDeltaY = ImGui.getMouseDragDeltaY(ImGuiMouseButton.Right);
      boolean isWindowHovered = ImGui.isWindowHovered();

      updateFromSourceTransform();

      if (!rightMouseDown)
      {
         dragging = false;
      }
      if (isWindowHovered && !dragging)
      {
         Line3DReadOnly pickRay = input.getPickRayInWorld();
         determineCurrentSelectionFromPickRay(pickRay);

         if (rightMouseDown)
         {
            if (closestCollisionSelection != null)
            {
               dragging = true;
               dragBucketX = 0.0f;
               dragBucketY = 0.0f;
               angularDragPlaneIntersectionPrevious.setToNaN();
            }
         }
      }
      if (dragging)
      {
         mouseDraggedX = mouseDragDeltaX - dragBucketX;
         mouseDraggedY = mouseDragDeltaY - dragBucketY;

         dragBucketX += mouseDraggedX;
         dragBucketY += mouseDraggedY;

         Line3DReadOnly pickRay = input.getPickRayInWorld();

         axisDragLine.getPoint().set(transform.getTranslation());
         axisDragLine.getDirection().set(Axis3D.Z);
         axisRotations[closestCollisionSelection.toAxis3D().ordinal()].transform(axisDragLine.getDirection());
         transform.getRotation().transform(axisDragLine.getDirection());

         if (closestCollisionSelection.isLinear())
         {
            axisDragLine.getPoint().set(closestCollision);
            axisDragPlane.set(axisDragLine.getPoint(), axisDragLine.getDirection());

            pickRay.closestPointsWith(axisDragLine, null, axisDragLineClosest);
            double distanceToMove = axisDragPlane.signedDistance(axisDragLineClosest);
            axisMoveVector.set(axisDragLine.getDirection());
            axisMoveVector.scale(distanceToMove);

            transform.getTranslation().add(axisMoveVector);
            closestCollision.add(axisMoveVector);
         }
         else if (closestCollisionSelection.isAngular())
         {
            if (angularDragPlaneIntersectionPrevious.containsNaN())
            {
               axisDragPlane.set(closestCollision, axisDragLine.getDirection());
               axisDragPlane.intersectionWith(axisDragLine, axisCollisionWithAngularPickPlane);
               axisDragPlane.getPoint().set(axisCollisionWithAngularPickPlane);

               angularDragPlaneIntersectionPrevious.set(closestCollision);
            }
            else
            {
               axisDragPlane.intersectionWith(angularDragPlaneIntersection, pickRay.getPoint(), pickRay.getDirection());

               clockHandVector.set(angularDragPlaneIntersection.getX() - axisDragPlane.getPoint().getX(),
                                   angularDragPlaneIntersection.getY() - axisDragPlane.getPoint().getY(),
                                   angularDragPlaneIntersection.getZ() - axisDragPlane.getPoint().getZ());
               previousClockHandVector.set(angularDragPlaneIntersectionPrevious.getX() - axisDragPlane.getPoint().getX(),
                                           angularDragPlaneIntersectionPrevious.getY() - axisDragPlane.getPoint().getY(),
                                           angularDragPlaneIntersectionPrevious.getZ() - axisDragPlane.getPoint().getZ());

               double deltaAngle = EuclidGeometryTools.angleFromFirstToSecondVector3D(previousClockHandVector.getX(),
                                                                                      previousClockHandVector.getY(),
                                                                                      previousClockHandVector.getZ(),
                                                                                      clockHandVector.getX(),
                                                                                      clockHandVector.getY(),
                                                                                      clockHandVector.getZ());

               if (!Double.isNaN(deltaAngle))
               {
                  crossProduct.cross(previousClockHandVector, clockHandVector);
                  if (crossProduct.dot(axisDragPlane.getNormal()) < 0.0)
                     deltaAngle = -deltaAngle;

                  axisAngleToRotateBy.set(axisDragPlane.getNormal(), deltaAngle);
                  axisAngleToRotateBy.transform(transform.getRotation());
               }

               angularDragPlaneIntersectionPrevious.set(angularDragPlaneIntersection);
            }
         }
      }

      // after things have been modified, update the derivative stuff
      updateFromSourceTransform();
   }

   private void updateFromSourceTransform()
   {
      for (Axis3D axis : Axis3D.values)
      {
         pose.set(transform);
         tempTransform.set(transform);
         tempTransform.appendOrientation(axisRotations[axis.ordinal()]);
         GDXTools.toGDX(tempTransform, linearControlModelInstances[axis.ordinal()].transform);
         GDXTools.toGDX(tempTransform, angularControlModelInstances[axis.ordinal()].transform);
      }
   }

   private void determineCurrentSelectionFromPickRay(Line3DReadOnly pickRay)
   {
      closestCollisionSelection = null;
      double closestCollisionDistance = Double.POSITIVE_INFINITY;

      // Optimization: could do one large sphere collision to avoid completely far off picks

      // collide tori
      for (Axis3D axis : Axis3D.values)
      {
         GDXTools.toEuclid(angularControlModelInstances[axis.ordinal()].transform, tempTransform);

         angularCollisionTorus.setToZero();
         angularCollisionTorus.setRadii(torusRadius.get(), torusTubeRadius.get());
         angularCollisionTorus.applyTransform(tempTransform);

         angularCollisionSphere.setToZero();
         angularCollisionSphere.setRadius(torusRadius.get() + torusTubeRadius.get());
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
            for (int i = 0; i < 100; i++)
            {
               interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);
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
               if (side == RobotSide.LEFT) // only show the cones in the positive direction
               {
                  tempPolytopeTransform.getTranslation().addZ(0.5 * arrowSpacing + arrowBodyLength);
                  tempTransform.transform(tempPolytopeTransform);

                  arrowHeadCollisionSphere.setToZero();
                  double arrowSurroundingSphereRadius;
                  if (arrowHeadRadius > (0.5 * arrowHeadLength))
                     arrowSurroundingSphereRadius = arrowHeadRadius / Math.sin(Math.atan(2.0 * arrowHeadRadius / arrowHeadLength));
                  else
                     arrowSurroundingSphereRadius = 0.5 * arrowHeadLength;
                  arrowHeadCollisionSphere.setRadius(arrowSurroundingSphereRadius);
                  arrowHeadCollisionSphere.getPosition().addZ(0.5 * arrowSpacing + arrowBodyLength + 0.5 * arrowHeadLength);
                  arrowHeadCollisionSphere.applyTransform(tempTransform);

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

                     for (int i = 0; i < 100; i++)
                     {
                        interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / 100.0);

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
                     }
                  }
               }
            }
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
   }


   public void renderImGuiTuner()
   {
      ImGui.begin(imGuiWindowName);

      ImGui.text("Use the right mouse button to manipulate the widget.");
      ImGui.text("X: " + mouseDraggedX);
      ImGui.text("Y: " + mouseDraggedY);

      if (ImGui.button("Reset"))
      {
         transform.setToZero();
      }

      ImGui.pushItemWidth(100.00f);
      boolean proportionsChanged = false;
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Torus radius"), torusRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Torus tube radius"), torusTubeRadius.getData(), 0.001f, 0.0f, 1000.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow length ratio"), arrowLengthRatio.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow head body length ratio"), arrowHeadBodyLengthRatio.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow head body radius ratio"), arrowHeadBodyRadiusRatio.getData(), 0.001f, 0.0f, 1.0f);
      proportionsChanged |= ImGui.dragFloat(ImGuiTools.uniqueLabel(this, "Arrow spacing factor"), arrowSpacingFactor.getData(), 0.001f, 0.0f, 1000.0f);
      ImGui.popItemWidth();

      if (proportionsChanged)
         recreateGraphics();

      ImGui.end();

      updateFromSourceTransform();
   }

   private void recreateGraphics()
   {
      arrowBodyRadius = (float) torusTubeRadius.get();
      arrowLength = arrowLengthRatio.get() * torusRadius.get();
      arrowBodyLength = (1.0 - arrowHeadBodyLengthRatio.get()) * arrowLength;
      arrowHeadRadius = arrowHeadBodyRadiusRatio.get() * arrowBodyRadius;
      arrowHeadLength = arrowHeadBodyLengthRatio.get() * arrowLength;
      arrowSpacing = arrowSpacingFactor.get() * (torusRadius.get() + torusTubeRadius.get());

      for (Axis3D axis : Axis3D.values)
      {
         if (linearControlModelInstances[axis.ordinal()] != null)
            linearControlModelInstances[axis.ordinal()].model.dispose();

         String axisName = axis.name().toLowerCase();

         Color color = axisColors[axis.ordinal()];
         ModelInstance arrow = GDXModelPrimitives.buildModelInstance(meshBuilder ->
         {
            // Euclid cylinders are defined from the center, but mesh builder defines them from the bottom
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing), color);
            meshBuilder.addCone(arrowHeadLength, arrowHeadRadius, new Point3D(0.0, 0.0, 0.5 * arrowSpacing + arrowBodyLength), color);
            meshBuilder.addCylinder(arrowBodyLength, arrowBodyRadius, new Point3D(0.0, 0.0, -0.5 * arrowSpacing), FLIP_180, color);
         }, axisName);
         arrow.materials.get(0).set(new BlendingAttribute(true, axisColors[axis.ordinal()].a));
         normalMaterials[axis.ordinal()] = new Material(arrow.materials.get(0));
         highlightedMaterials[axis.ordinal()] = new Material();
         Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
         highlightedMaterials[axis.ordinal()].set(TextureAttribute.createDiffuse(paletteTexture));
         highlightedMaterials[axis.ordinal()].set(new BlendingAttribute(true, axisSelectedColors[axis.ordinal()].a));
         GDXTools.toGDX(axisRotations[axis.ordinal()], arrow.transform);
         linearControlModelInstances[axis.ordinal()] = arrow;
      }
      for (Axis3D axis : Axis3D.values)
      {
         if (angularControlModelInstances[axis.ordinal()] != null)
            angularControlModelInstances[axis.ordinal()].model.dispose();

         String axisName = axis.name().toLowerCase();

         int resolution = 25;
         ModelInstance ring = GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                          meshBuilder.addArcTorus(0.0, Math.PI * 2.0f, torusRadius.get(), torusTubeRadius.get(), resolution, axisColors[axis.ordinal()]), axisName);
         ring.materials.get(0).set(new BlendingAttribute(true, axisColors[axis.ordinal()].a));
         GDXTools.toGDX(axisRotations[axis.ordinal()], ring.transform);
         angularControlModelInstances[axis.ordinal()] = ring;
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

   public Pose3DReadOnly getPose()
   {
      return pose;
   }

   // TODO: Make this transform the ground truth and give the pose as needed only
   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public static Mesh angularHighlightMesh(double majorRadius, double minorRadius)
   {
      return tetrahedronRingMesh(1.75 * minorRadius, 1.25 * minorRadius, 5);
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
