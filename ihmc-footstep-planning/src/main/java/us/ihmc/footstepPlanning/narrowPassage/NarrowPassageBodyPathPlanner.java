package us.ihmc.footstepPlanning.narrowPassage;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NarrowPassageBodyPathPlanner
{
   // can make this dependent on the path length later
   private static int numberOfWaypoints = 15;
   private static int maxOrientationAdjustmentIterations = 20;
   private static int maxPoseAdjustmentIterations = 20;

   private static final double bodyDepth = 0.4;
   private static final double bodyWidth = 0.7;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoBoolean collisionFound = new YoBoolean("collisionFound", registry);
   private final VisibilityGraphsParametersReadOnly parameters;
   private final boolean visualize;

   private PlanarRegionsList planarRegionsList;

   private final YoFramePose3D startPose = new YoFramePose3D("startPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D endPose = new YoFramePose3D("endPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D[] waypoints = new YoFramePose3D[numberOfWaypoints];
   private final List<BodyCollisionPoint> bodyCollisionPoints = new ArrayList<>();

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private static final double collisionGradientScale = 0.5;
   private final List<Vector3D> collisionGradients = new ArrayList<>();
   private final List<Vector3D> convolvedGradients = new ArrayList<>();
   private final List<Vector3D> yaws = new ArrayList<>();
   private final List<Vector3D> convolvedYaws = new ArrayList<>();
   private final TDoubleArrayList convolutionWeights = new TDoubleArrayList();

   // for visualization only
   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameVector3D startOrientation = new YoFrameVector3D("startOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D endOrientation = new YoFrameVector3D("endOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final BagOfBalls waypointPointGraphic;

   public NarrowPassageBodyPathPlanner(VisibilityGraphsParametersReadOnly parameters,
                                       FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                       YoRegistry parentRegistry)
   {
      this(parameters, footstepPlannerParameters, null, null, parentRegistry);
   }

   public NarrowPassageBodyPathPlanner(VisibilityGraphsParametersReadOnly parameters,
                                       FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                       YoGraphicsListRegistry graphicsListRegistry,
                                       TickAndUpdatable tickAndUpdatable,
                                       YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.graphicsListRegistry = graphicsListRegistry;
      this.tickAndUpdatable = tickAndUpdatable;

      visualize = graphicsListRegistry != null;
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         waypoints[i] = new YoFramePose3D("waypoint" + i, ReferenceFrame.getWorldFrame(), registry);
         bodyCollisionPoints.add(new BodyCollisionPoint(i, footstepPlannerParameters, graphicsListRegistry, registry));
         collisionGradients.add(new Vector3D());
         convolvedGradients.add(new Vector3D());
         yaws.add(new Vector3D());
         convolvedYaws.add(new Vector3D());
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (visualize)
      {
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());

         YoGraphicPosition startPosition = new YoGraphicPosition("startPositionGraphic", startPose.getPosition(), 0.05, YoAppearance.Green());
         YoGraphicPosition endPosition = new YoGraphicPosition("endPositionGraphic", endPose.getPosition(), 0.05, YoAppearance.Red());

         YoGraphicVector startOrientationGraphic = new YoGraphicVector("startOrientationGraphic", startPose.getPosition(), startOrientation, 0.3);
         YoGraphicVector endOrientationGraphic = new YoGraphicVector("endOrientationGraphic", endPose.getPosition(), endOrientation, 0.3);

         waypointPointGraphic = new BagOfBalls(numberOfWaypoints, 0.03, YoAppearance.White(), registry, graphicsListRegistry);

         graphicsList.add(startPosition);
         graphicsList.add(endPosition);
         graphicsList.add(startOrientationGraphic);
         graphicsList.add(endOrientationGraphic);

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
      }
      else
      {
         waypointPointGraphic = null;
      }
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public List<Pose3DReadOnly> computePlan(Pose3DReadOnly startPose, Pose3DReadOnly endPose)
   {
      this.startPose.set(startPose);
      this.endPose.set(endPose);

      if (visualize)
      {
         setStartAndEndGraphics();
      }

      // initialize the waypoints with simple interpolation
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double alpha = (i + 1) / (double) (numberOfWaypoints + 1);
         waypoints[i].interpolate(startPose, endPose, alpha);
      }
      updateWaypointPointGraphics();

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

   private void setStartAndEndGraphics()
   {
      Vector3D forwardDirection = new Vector3D(1.0, 0.0, 0.0);
      startPose.getOrientation().transform(forwardDirection);
      startOrientation.set(forwardDirection);

      forwardDirection.set(1.0, 0.0, 0.0);
      endPose.getOrientation().transform(forwardDirection);
      endOrientation.set(forwardDirection);

      tickAndUpdatable.tickAndUpdate();
   }

   private void initializeBodyPoints()
   {
      Vector3D vectorToNextWaypoint = new Vector3D();
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         // TODO Fix orientation of collision boxes during init
         if (i < numberOfWaypoints - 1)
            vectorToNextWaypoint.set(waypoints[i + 1].getX() - waypoints[i].getX(),
                                     waypoints[i + 1].getY() - waypoints[i].getY(),
                                     waypoints[i + 1].getZ() - waypoints[i].getZ());
         bodyCollisionPoints.get(i).initializeWaypointAdjustmentFrame(vectorToNextWaypoint);
         bodyCollisionPoints.get(i).initialize(waypoints[i]);

         convolutionWeights.add(exp(0.65, i));
      }
      if (visualize)
      {
         /* Show one collision box at a time */
         for (int boxToShow = 0; boxToShow < numberOfWaypoints; boxToShow++)
         {
            bodyCollisionPoints.forEach(kp -> kp.updateGraphics(false));
            bodyCollisionPoints.get(boxToShow).updateGraphics(true);
            tickAndUpdatable.tickAndUpdate();
         }
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
               LogTools.info(k);
               LogTools.info("collision gradient " + collisionGradients.get(k));
               LogTools.info("convolved gradient" + convolvedGradients.get(j));
               LogTools.info("scale " + scale);
            }

            bodyCollisionPoints.get(j).project(convolvedGradients.get(j));
            bodyCollisionPoints.get(j).shiftWaypoint(convolvedGradients.get(j));

            if (visualize)
            {
               bodyCollisionPoints.get(j).updateGraphics(bodyCollisionPoints.get(j).getCollisionResult().areShapesColliding());
            }
         }

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
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
               bodyCollisionPoints.get(j).updateGraphics(bodyCollisionPoints.get(j).getCollisionResult().areShapesColliding());

            }
            else
            {
               yaws.get(j).setToZero();
            }

            if (visualize)
            {
               bodyCollisionPoints.get(j).updateGraphics(bodyCollisionPoints.get(j).getCollisionResult().areShapesColliding());
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

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
         }

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
         if (visualize) bodyCollisionPoints.get(i).updateGraphics(true);
      }
      updateWaypointPointGraphics();
   }

   private void updateWaypointPointGraphics()
   {
      if (!visualize)
      {
         return;
      }

      waypointPointGraphic.reset();
      for (int i = 0; i < waypoints.length; i++)
      {
         YoFramePose3D waypoint = waypoints[i];
         waypointPointGraphic.setBall(waypoint.getPosition());
      }

      tickAndUpdatable.tickAndUpdate();
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
