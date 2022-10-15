package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Map;

public class BodyPathLSTraversibilityCalculator
{
   private static final int minTraversibleSteps = 3;

   private final FootstepPlannerParametersReadOnly parameters;
   private final Pose2D bodyPose = new Pose2D();
   private final Pose2D stepPose = new Pose2D();

   private final HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
   private final Map<BodyPathLatticePoint, Double> gridHeightMap;

   private final YoDouble xBody, yBody, yawBody;
   private BodyPathLatticePoint startNode;

   private final SideDependentList<YoDouble> xStep;
   private final SideDependentList<YoDouble> yStep;
   private final SideDependentList<YoDouble> yawStep;
   private final SideDependentList<YoInteger> validSteps;

   private final SideDependentList<YoDouble> rmsAlpha;
   private final SideDependentList<YoDouble> areaAlpha;
   private final SideDependentList<YoDouble> inclineAlpha;

   private final YoDouble leftTraversibility, rightTraversibility;

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();
   private final TDoubleArrayList traversibilityCosts = new TDoubleArrayList();
   private HeightMapData heightMapData;

   public BodyPathLSTraversibilityCalculator(FootstepPlannerParametersReadOnly parameters,
                                             SideDependentList<ConvexPolygon2D> footPolygons,
                                             Map<BodyPathLatticePoint, Double> gridHeightMap,
                                             YoRegistry registry)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.gridHeightMap = gridHeightMap;

      xBody = new YoDouble("xBody", registry);
      yBody = new YoDouble("yBody", registry);
      yawBody = new YoDouble("yawBody", registry);

      xStep = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "XStep", registry));
      yStep = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "YStep", registry));
      yawStep = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "YawStep", registry));

      rmsAlpha = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "RmsAlpha", registry));
      areaAlpha = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "AreaAlpha", registry));
      inclineAlpha = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "InclineAlpha", registry));
      validSteps = new SideDependentList<>(side -> new YoInteger(side.getCamelCaseNameForStartOfExpression() + "ValidSteps", registry));

      leftTraversibility = new YoDouble("leftTraversibility", registry);
      rightTraversibility = new YoDouble("rightTraversibility", registry);

      xOffsets.add(0.0);
      xOffsets.add(0.06);
      xOffsets.add(-0.06);
      xOffsets.add(0.12);
      xOffsets.add(-0.12);

      yOffsets.add(0.0);
      yOffsets.add(0.08);

      yawOffsets.add(0.0);
      yawOffsets.add(Math.toRadians(20.0));
      yawOffsets.add(Math.toRadians(-20.0));
   }

   public void setHeightMap(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public void initialize(BodyPathLatticePoint startNode)
   {
      this.startNode = startNode;
   }

   public double computeTraversibilityIndicator(BodyPathLatticePoint node, BodyPathLatticePoint parentNode)
   {
      double yaw = Math.atan2(node.getY() - parentNode.getY(), node.getX() - parentNode.getX());
      bodyPose.set(node.getX(), node.getY(), yaw);

      xBody.set(bodyPose.getX());
      yBody.set(bodyPose.getY());
      yawBody.set(bodyPose.getYaw());

      double nodeHeight = gridHeightMap.get(parentNode);
      double parentHeight = gridHeightMap.get(parentNode);

      leftTraversibility.set(compute(RobotSide.LEFT, parentHeight));
      rightTraversibility.set(compute(RobotSide.RIGHT, parentHeight));

      double alphaNode0 = 0.2;
      double alphaNode1 = 0.05;
      double alphaEdge = 0.5;

      bodyPose.set(parentNode.getX(), parentNode.getY(), yaw);
      double previousLeftTraversibility = 0.0;
      double previousRightTraversibility = 0.0;

      if (!parentNode.equals(startNode))
      {
         previousLeftTraversibility = compute(RobotSide.LEFT, nodeHeight);
         previousRightTraversibility = compute(RobotSide.RIGHT, nodeHeight);
      }

               // Node cost is scored by having one side that has good footholds
      return alphaNode0 * Math.min(leftTraversibility.getValue(), rightTraversibility.getValue()) +
//             alphaNode1 * Math.max(leftTraversibility.getValue(), rightTraversibility.getValue()) +
             // Edge cost is scored by making sure not all good footholds are on the same side
             alphaEdge * Math.min(leftTraversibility.getValue() * previousRightTraversibility, rightTraversibility.getValue() * previousLeftTraversibility);
   }

   public boolean isTraversible()
   {
      return validSteps.get(RobotSide.LEFT).getValue() >= minTraversibleSteps || validSteps.get(RobotSide.RIGHT).getValue() >= minTraversibleSteps;
   }

   /**
    * Returns a traversibility indicator between 0 and 1 where 0 is planar and level and 1 is non-planar and inclined.
    */
   private double compute(RobotSide side, double parentHeight)
   {
      traversibilityCosts.clear();
      validSteps.get(side).set(0);

      double fullFootholdArea = footPolygons.get(side).getArea();
      double maxAreaToPenalize = 0.9;
      double minAreaThreshold = 0.65;

      double minSurfaceInclineToPenalize = Math.toRadians(10.0);
      double maxSurfaceIncline = Math.toRadians(35.0);

      TDoubleArrayList xSteps = new TDoubleArrayList();
      TDoubleArrayList ySteps = new TDoubleArrayList();
      TDoubleArrayList yawSteps = new TDoubleArrayList();
      TDoubleArrayList rmsAlphas = new TDoubleArrayList();
      TDoubleArrayList areaAlphas = new TDoubleArrayList();
      TDoubleArrayList inclineAlphas = new TDoubleArrayList();

      Pose2D rotatedBodyPose = new Pose2D();

      for (int ti = 0; ti < yawOffsets.size(); ti++)
      {
         rotatedBodyPose.set(bodyPose);
         rotatedBodyPose.appendRotation(yawOffsets.get(ti));

         for (int xi = 0; xi < xOffsets.size(); xi++)
         {
            for (int yi = 0; yi < yOffsets.size(); yi++)
            {
               stepPose.set(rotatedBodyPose);
               stepPose.appendTranslation(0.0, side.negateIfRightSide(0.5 * parameters.getIdealFootstepWidth()));
               stepPose.appendTranslation(xOffsets.get(xi), side.negateIfRightSide(yOffsets.get(yi)));
               stepPose.appendRotation(side.negateIfRightSide(yawOffsets.get(ti)));

               RigidBodyTransform transform = new RigidBodyTransform();
               stepPose.get(transform);

               ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(side));
               footPolygon.applyTransform(transform);

               double heightWindow = 0.2;
               RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(footPolygon,
                                                                                 heightMapData,
                                                                                 parameters.getHeightMapSnapThreshold(),
                                                                                 parentHeight - heightWindow);
               if (snapTransform == null)
               {
                  if (xi == 0 && yi == 0 && ti == 0)
                  {
                     xStep.get(side).set(stepPose.getX());
                     yStep.get(side).set(stepPose.getY());
                     yawStep.get(side).set(stepPose.getYaw());
                     this.rmsAlpha.get(side).set(Double.NaN);
                     this.areaAlpha.get(side).set(Double.NaN);
                     this.inclineAlpha.get(side).set(Double.NaN);
                  }

                  continue;
               }
               else
               {
                  double rmsAlpha = Math.max(0.0,
                                             (snapper.getRMSError() - parameters.getRMSMinErrorToPenalize()) / (parameters.getRMSErrorThreshold()
                                                                                                                - parameters.getRMSMinErrorToPenalize()));
                  double areaAlpha = Math.max(0.0, 1.0 - (snapper.getArea() / fullFootholdArea - minAreaThreshold) / (maxAreaToPenalize - minAreaThreshold));
                  double inclineAlpha = Math.max(0.0,
                                                 (Math.acos(snapTransform.getM22()) - minSurfaceInclineToPenalize) / (maxSurfaceIncline
                                                                                                                      - minSurfaceInclineToPenalize));

                  if (xi == 0 && yi == 0 && ti == 0)
                  {
                     xStep.get(side).set(stepPose.getX());
                     yStep.get(side).set(stepPose.getY());
                     yawStep.get(side).set(stepPose.getYaw());
                     this.rmsAlpha.get(side).set(rmsAlpha);
                     this.areaAlpha.get(side).set(areaAlpha);
                     this.inclineAlpha.get(side).set(inclineAlpha);
                  }

                  if (rmsAlpha < 1.0 && areaAlpha < 1.0 && inclineAlpha < 1.0)
                  {
                     validSteps.get(side).increment();
                  }

                  xSteps.add(stepPose.getX());
                  ySteps.add(stepPose.getY());
                  yawSteps.add(stepPose.getYaw());
                  rmsAlphas.add(rmsAlpha);
                  areaAlphas.add(areaAlpha);
                  inclineAlphas.add(inclineAlpha);

                  traversibilityCosts.add((rmsAlpha + areaAlpha + inclineAlpha) / 3.0);
               }
            }
         }
      }

      validSteps.get(side).set(traversibilityCosts.size());
      if (traversibilityCosts.isEmpty())
      {
         return 3.0;
      }

      int minIndex = traversibilityCosts.indexOf(traversibilityCosts.min());
      xStep.get(side).set(xSteps.get(minIndex));
      yStep.get(side).set(ySteps.get(minIndex));
      yawStep.get(side).set(yawSteps.get(minIndex));
      rmsAlpha.get(side).set(rmsAlphas.get(minIndex));
      areaAlpha.get(side).set(areaAlphas.get(minIndex));
      inclineAlpha.get(side).set(inclineAlphas.get(minIndex));

      traversibilityCosts.sort();
      double sampledTraversibility = 0.0;

      int samples = Math.min(minTraversibleSteps, traversibilityCosts.size());
      for (int i = 0; i < samples; i++)
      {
         sampledTraversibility += traversibilityCosts.get(i);
      }

      return sampledTraversibility / samples;
   }
}
