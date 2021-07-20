package us.ihmc.footstepPlanning.narrowPassage;

import gnu.trove.list.array.TDoubleArrayList;
import sun.rmi.runtime.Log;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NarrowPassageAdjuster
{
   private static int numberOfWaypoints;
   private static int maxOrientationAdjustmentIterations = 20;
   private static int maxPoseAdjustmentIterations = 20;

   private static final double bodyDepth = 0.4;
   private static final double bodyWidth = 0.7;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoBoolean collisionFound = new YoBoolean("collisionFound", registry);
   private final VisibilityGraphsParametersReadOnly parameters;
   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;

   private PlanarRegionsList planarRegionsList;

   private FramePose3D startPose = new FramePose3D();
   private FramePose3D endPose = new FramePose3D();
   private FramePose3D[] waypoints;
   private final List<BodyCollisionPoint> bodyCollisionPoints = new ArrayList<>();

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private static final double collisionGradientScale = 0.5;
   private final List<Vector3D> collisionGradients = new ArrayList<>();
   private final List<Vector3D> convolvedGradients = new ArrayList<>();
   private final List<Vector3D> yaws = new ArrayList<>();
   private final List<Vector3D> convolvedYaws = new ArrayList<>();
   private final TDoubleArrayList convolutionWeights = new TDoubleArrayList();

   public NarrowPassageAdjuster(VisibilityGraphsParametersReadOnly parameters,
                                FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                YoRegistry parentRegistry)
   {
      this(parameters, footstepPlannerParameters, null, null, parentRegistry);
   }

   public NarrowPassageAdjuster(VisibilityGraphsParametersReadOnly parameters,
                                FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                FramePose3D[] waypoints,
                                YoGraphicsListRegistry graphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.waypoints = waypoints;
      this.graphicsListRegistry = graphicsListRegistry;
      this.footstepPlannerParameters = footstepPlannerParameters;

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public List<Pose3DReadOnly> makeAdjustments()
   {
      initializeBodyPoints();
      adjustBodyPointPositions();
      adjustBodyPointOrientations();
      recomputeWaypoints();

      List<Pose3DReadOnly> bodyPath = new ArrayList<>();
      bodyPath.add(startPose);
      bodyPath.addAll(Arrays.asList(waypoints));
      bodyPath.add(endPose);

      return bodyPath;
   }

   private void initializeBodyPoints()
   {
      Vector3D vectorToNextWaypoint = new Vector3D();
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         bodyCollisionPoints.add(new BodyCollisionPoint(i, footstepPlannerParameters, graphicsListRegistry, registry));
         collisionGradients.add(new Vector3D());
         convolvedGradients.add(new Vector3D());
         yaws.add(new Vector3D());
         convolvedYaws.add(new Vector3D());
         convolutionWeights.add(exp(0.65, i));
         if (i < numberOfWaypoints - 1)
            vectorToNextWaypoint.set(waypoints[i + 1].getX() - waypoints[i].getX(),
                                     waypoints[i + 1].getY() - waypoints[i].getY(),
                                     waypoints[i + 1].getZ() - waypoints[i].getZ());
         bodyCollisionPoints.get(i).initializeWaypointAdjustmentFrame(vectorToNextWaypoint);
         bodyCollisionPoints.get(i).initialize(waypoints[i]);
      }
   }

   private void adjustBodyPointPositions()
   {
      collisionFound.set(false);

      for (int i = 0; i < maxPoseAdjustmentIterations; i++)
      {
         boolean intersectionFound = false;
         for (int j = 0; j < numberOfWaypoints; j++)
         {
            BodyCollisionPoint bodyCollisionPoint = bodyCollisionPoints.get(j);

            boolean collisionDetected = bodyCollisionPoint.doCollisionCheck(collisionDetector, planarRegionsList);
            if (collisionDetected)
            {
               collisionFound.set(true);
               intersectionFound = true;

               EuclidShape3DCollisionResult collisionResult = bodyCollisionPoint.getCollisionResult();
               collisionGradients.get(j).sub(collisionResult.getPointOnB(), collisionResult.getPointOnA());
               collisionGradients.get(j).scale(collisionGradientScale);
               bodyCollisionPoints.get(j).project(collisionGradients.get(j));
            }
            else
            {
               collisionGradients.get(j).setToZero();
               continue;
            }

            //            if (bodyCollisionPoints.get(j).getCollisionResult().areShapesColliding())
            //            {
            //               double scale = bodyCollisionPoints.get(j).computeMaximumDisplacementScale(collisionGradients.get(j));
            //               collisionGradients.get(j).scale(scale);
            //            }
         }

         for (int j = 0; j < numberOfWaypoints; j++)
         {
            convolvedGradients.get(j).setToZero();
            for (int k = 0; k < numberOfWaypoints; k++)
            {
               int indexDifference = Math.abs(j - k);
               double scale = convolutionWeights.get(indexDifference);
               scaleAdd(convolvedGradients.get(j), scale, collisionGradients.get(k));
            }

            bodyCollisionPoints.get(j).project(convolvedGradients.get(j));
            bodyCollisionPoints.get(j).shiftWaypoint(convolvedGradients.get(j));
         }

         if (!intersectionFound)
         {
            break;
         }
      }
   }

   private void adjustBodyPointOrientations()
   {
      // Rotation direction determined by first collision vector TODO find a better way to determine this
      boolean rotationDirectionDetermined = false;
      int rotationDirection = 0;

      for (int i = 0; i < maxOrientationAdjustmentIterations; i++)
      {
         boolean intersectionFound = false;

         // Adjust waypoint orientations
         for (int j = 0; j < numberOfWaypoints; j++)
         {
            BodyCollisionPoint bodyCollisionPoint = bodyCollisionPoints.get(j);

            boolean collisionDetected = bodyCollisionPoint.doCollisionCheck(collisionDetector, planarRegionsList);
            if (collisionDetected)
            {
               EuclidShape3DCollisionResult collisionResult = bodyCollisionPoint.getCollisionResult();
               Vector3D collisionVector = new Vector3D();
               collisionVector.sub(collisionResult.getPointOnB(), collisionResult.getPointOnA());
               if (!rotationDirectionDetermined)
               {
                  rotationDirection = bodyCollisionPoint.getRotationDirection(collisionVector);
                  rotationDirectionDetermined = true;
               }
               yaws.set(j, collisionVector);
               intersectionFound = true;

               bodyCollisionPoints.get(j).adjustOrientation(rotationDirection, yaws.get(j));
            }
            else
            {
               yaws.get(j).setToZero();
            }
         }

         // Smooth out orientations along path (convolve yaws) we might not want to do this
         //         for (int j = 0; j < numberOfWaypoints; j++)
         //         {
         //            convolvedYaws.get(j).setToZero();
         //            for (int k = 0; k < numberOfWaypoints; k++)
         //            {
         //               int indexDifference = Math.abs(j - k);
         //               double scale = convolutionWeights.get(indexDifference);
         //               scaleAdd(convolvedYaws.get(j), scale, yaws.get(k));
         //            }
         //
         //            bodyCollisionPoints.get(j).adjustOrientation(rotationDirection, convolvedYaws.get(j));
         //
         //            if (visualize)
         //            {
         //               bodyCollisionPoints.get(j).updateGraphics(bodyCollisionPoints.get(j).getCollisionResult().areShapesColliding());
         //            }
         //         }

         if (!intersectionFound)
         {
            break;
         }
      }
   }

   private void recomputeWaypoints()
   {
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         waypoints[i].set(bodyCollisionPoints.get(i).getOptimizedWaypoint());
         waypoints[i].getOrientation().set(bodyCollisionPoints.get(i).getOptimizedWaypoint().getOrientation());
      }
   }

   public void setWaypoints(FramePose3D[] waypoints)
   {
      LogTools.info("Number of (input) waypoints: " + waypoints.length);
      this.startPose.set(waypoints[0]);
      this.endPose.set(waypoints[waypoints.length - 1]);

      // Omit start and end pose from waypoints
      this.waypoints = Arrays.copyOfRange(waypoints, 1, waypoints.length - 1);
      this.numberOfWaypoints = this.waypoints.length;

      LogTools.info("Number of waypoints: " + this.waypoints.length);
   }

   /*
    * Different from the Vector3DBasics.scaleAdd, which scales the mutated vector
    * a = a + alpha * b
    */
   static void scaleAdd(Vector3DBasics vectorA, double alpha, Vector3DReadOnly vectorB)
   {
      vectorA.addX(alpha * vectorB.getX());
      vectorA.addY(alpha * vectorB.getY());
      vectorA.addZ(alpha * vectorB.getZ());
   }

   /* exponent function assuming non-negative positive exponent */
   static double exp(double base, int exponent)
   {
      double value = 1.0;
      int i = 0;

      while (i < exponent)
      {
         value *= base;
         i++;
      }

      return value;
   }
}
