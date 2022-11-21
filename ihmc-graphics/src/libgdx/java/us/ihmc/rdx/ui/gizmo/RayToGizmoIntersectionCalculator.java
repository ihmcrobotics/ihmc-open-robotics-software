package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.Material;
import imgui.type.ImBoolean;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RayToGizmoIntersectionCalculator
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private float torusRadius;
   private float torusCameraSize;
   private float torusTubeRadiusRatio;
   private float arrowLengthRatio;
   private float arrowHeadBodyLengthRatio;
   private float arrowHeadBodyRadiusRatio;
   private float arrowSpacingFactor;
   private final ImBoolean resizeAutomatically = new ImBoolean(true);
   private double arrowBodyRadius;
   private double arrowLength;
   private double arrowBodyLength;
   private double arrowHeadRadius;
   private double arrowHeadLength;
   private double arrowSpacing;
   private final Material[] normalMaterials = new Material[3];
   private final Material[] highlightedMaterials = new Material[3];
   private final Axis3DRotations axisRotations = new Axis3DRotations();
//   private final DynamicLibGDXModel[] arrowModels = new DynamicLibGDXModel[3];
//   private final DynamicLibGDXModel[] torusModels = new DynamicLibGDXModel[3];
   private final Point3D closestCollision = new Point3D();
   private SixDoFSelection closestCollisionSelection;
   private double closestCollisionDistance;
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final DiscreteTorusRayIntersection torusIntersection = new DiscreteTorusRayIntersection();
   private final DiscreteArrowRayIntersection arrowIntersection = new DiscreteArrowRayIntersection();

   public RayToGizmoIntersectionCalculator(float torusRadius,
                                           float torusCameraSize,
                                           float torusTubeRadiusRatio,
                                           float arrowLengthRatio,
                                           float arrowHeadBodyLengthRatio,
                                           float arrowHeadBodyRadiusRatio,
                                           float arrowSpacingFactor)
   {
      updateGizmoDimensions(torusRadius,
                            torusCameraSize,
                            torusTubeRadiusRatio,
                            arrowLengthRatio,
                            arrowHeadBodyLengthRatio,
                            arrowHeadBodyRadiusRatio,
                            arrowSpacingFactor);
   }


   public void updateGizmoDimensions(float torusRadius,
                                     float torusCameraSize,
                                     float torusTubeRadiusRatio,
                                     float arrowLengthRatio,
                                     float arrowHeadBodyLengthRatio,
                                     float arrowHeadBodyRadiusRatio,
                                     float arrowSpacingFactor)
   {
      this.torusRadius = torusRadius;
      this.torusCameraSize = torusCameraSize;
      this.torusTubeRadiusRatio = torusTubeRadiusRatio;
      this.arrowLengthRatio = arrowLengthRatio;
      this.arrowHeadBodyLengthRatio = arrowHeadBodyLengthRatio;
      this.arrowHeadBodyRadiusRatio = arrowHeadBodyRadiusRatio;
      this.arrowSpacingFactor = arrowSpacingFactor;
      this.arrowBodyRadius = torusTubeRadiusRatio * torusRadius;
      this.arrowLength = arrowLengthRatio * torusRadius;
      this.arrowBodyLength = (1.0 - arrowHeadBodyLengthRatio) * arrowLength;
      this.arrowHeadRadius = arrowHeadBodyRadiusRatio * arrowBodyRadius;
      this.arrowHeadLength = arrowHeadBodyLengthRatio * arrowLength;
      this.arrowSpacing = arrowSpacingFactor * (torusRadius + (torusTubeRadiusRatio * torusRadius));
   }

   public void determineCollisionSelectionFromRay(Line3DReadOnly pickRay,
                                                  RigidBodyTransform gizmoTransformToWorld,
                                                  RigidBodyTransform gizmoTempTransformToWorld,
                                                  DynamicLibGDXModel[] torusModels,
                                                  DynamicLibGDXModel[] arrowModels)
   {
      closestCollisionSelection = null;
      closestCollisionDistance = Double.POSITIVE_INFINITY;

      // Optimization: Do one large sphere collision to avoid completely far off picks
      boundingSphereIntersection.update(1.5 * torusRadius, gizmoTransformToWorld);
      if (boundingSphereIntersection.intersect(pickRay))
      {
         // collide tori
         for (Axis3D axis : Axis3D.values)
         {
            LibGDXTools.toEuclid(torusModels[axis.ordinal()].getOrCreateModelInstance().transform, gizmoTempTransformToWorld);
            // TODO: Only update when shape changes?
            torusIntersection.update(torusRadius, torusTubeRadiusRatio * torusRadius, gizmoTempTransformToWorld);
            double distance = torusIntersection.intersect(pickRay, 100);
            if (!Double.isNaN(distance) && distance < closestCollisionDistance)
            {
               closestCollisionDistance = distance;
               closestCollisionSelection = SixDoFSelection.toAngularSelection(axis);
               closestCollision.set(torusIntersection.getClosestIntersection());
            }
         }

         // collide arrows
         for (Axis3D axis : Axis3D.values)
         {
            LibGDXTools.toEuclid(arrowModels[axis.ordinal()].getOrCreateModelInstance().transform, gizmoTempTransformToWorld);

            for (RobotSide side : RobotSide.values)
            {
               double zOffset = side.negateIfRightSide(0.5 * arrowSpacing + 0.5 * arrowBodyLength);
               // TODO: Only update when shape changes?
               arrowIntersection.update(arrowBodyLength, arrowBodyRadius, arrowHeadRadius, arrowHeadLength, zOffset, gizmoTempTransformToWorld);
               double distance = arrowIntersection.intersect(pickRay, 100, side == RobotSide.LEFT); // only show the cones in the positive direction

               if (!Double.isNaN(distance) && distance < closestCollisionDistance)
               {
                  closestCollisionDistance = distance;
                  closestCollisionSelection = SixDoFSelection.toLinearSelection(axis);
                  closestCollision.set(arrowIntersection.getIntersection());
               }
            }
         }
      }
   }

   public SixDoFSelection getClosestCollisionSelection()
   {
      return this.closestCollisionSelection;
   }

   public double getClosestCollisionDistance()
   {
      return this.closestCollisionDistance;
   }

   public Point3D getClosestCollision()
   {
      return this.closestCollision;
   }
}
