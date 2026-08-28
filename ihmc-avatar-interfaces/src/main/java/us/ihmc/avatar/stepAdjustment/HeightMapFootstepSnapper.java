package us.ihmc.avatar.stepAdjustment;

import static us.ihmc.footstepPlanning.polygonSnapping.PolygonSnapperTools.*;

import controller_msgs.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeightMapFootstepSnapper implements FootstepAdjustment
{
   // When snapping, the foot polygon is expanded by this amount to improve the surface normal detection
   private static final double FOOT_INFLATION_DISTANCE = 0.08;
   // Only heights within this vertical distance of the highest point are used in the snap computation
   private static final double MAXIMUM_HEIGHT_DELTA_TO_CONSIDER = 0.2;
   // Only heights within this vertical distance of the highest point are used in the snap computation
   private static final double MAXIMUM_HEIGHT_ABOVE_STANCE_TO_CONSIDER = 0.3;

   private HeightMapCommand heightMapCommand;
   private final SideDependentList<ConvexPolygon2DReadOnly> footPolygons = new SideDependentList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private final FramePose3D tempPose = new FramePose3D();
   private final ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
   private final RecyclingArrayList<Point3D> pointsInsidePolygon = new RecyclingArrayList<>(Point3D.class);
   private final RecyclingArrayList<Point3D> pointsInsidePolygonFiltered = new RecyclingArrayList<>(Point3D.class);
   private final YoBoolean hasHeightMap;
   private final YoBoolean snapToHeightMap;
   private final YoBoolean includePitchAndRoll;
   private final YoDouble heightOfNewPose;
   private final YoDouble maxHeightYoVariable;
   private final YoDouble maximumHeightDeltaToConsider;
   private final YoDouble maximumHeightAboveStanceToConsider;

   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public HeightMapFootstepSnapper(SideDependentList<ConvexPolygon2D> footPolygons, YoRegistry registry)
   {
      this.hasHeightMap = new YoBoolean("hasHeightMap", registry);
      this.snapToHeightMap = new YoBoolean("snapToHeightMap", registry);
      this.includePitchAndRoll = new YoBoolean("includePitchAndRoll", registry);

      heightOfNewPose = new YoDouble("HeightMapFootstepSnallerHeightOfNewPose", registry);
      maxHeightYoVariable = new YoDouble("HeightMapFootstepSnapperMaxHeight", registry);

      maximumHeightDeltaToConsider = new YoDouble("maximumHeightDeltaToConsider", registry);
      maximumHeightDeltaToConsider.set(MAXIMUM_HEIGHT_DELTA_TO_CONSIDER);

      maximumHeightAboveStanceToConsider = new YoDouble("maximumHeightAboveStanceToConsider", registry);
      maximumHeightAboveStanceToConsider.set(MAXIMUM_HEIGHT_ABOVE_STANCE_TO_CONSIDER);

      hasHeightMap.set(false);

      ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D();
         polygonScaler.scaleConvexPolygon(footPolygons.get(robotSide), -Math.abs(FOOT_INFLATION_DISTANCE), scaledFootPolygon);
         this.footPolygons.put(robotSide, scaledFootPolygon);
      }
   }

   @Override
   public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedPoseToPack)
   {
      adjustedPoseToPack.getLocation().getPoint().set(footstepPose.getPosition(), stanceFootPose.getZ());
      adjustedPoseToPack.getOrientation().getQuaternion().set(footstepPose.getOrientation());
      hasHeightMap.set(heightMapCommand != null);

      if (!hasHeightMap.getValue() || !snapToHeightMap.getValue())
      {
         return true;
      }

      double minimumZToConsider = stanceFootPose.getZ() - maximumHeightAboveStanceToConsider.getDoubleValue();
      double maximumZToConsider = stanceFootPose.getZ() + maximumHeightAboveStanceToConsider.getDoubleValue();
      double maximumHeight = -Double.MAX_VALUE;

      polygonToSnap.set(footPolygons.get(footSide));
      tempPose.set(footstepPose);
      polygonToSnap.applyTransform(tempPose, false);

      pointsInsidePolygon.clear();
      pointsInsidePolygonFiltered.clear();

      for (double x = polygonToSnap.getMinX(); x <= polygonToSnap.getMaxX(); x += heightMapCommand.getCellSize())
      {
         for (double y = polygonToSnap.getMinY(); y <= polygonToSnap.getMaxY(); y += heightMapCommand.getCellSize())
         {
            if (!polygonToSnap.isPointInside(x, y))
            {
               continue;
            }

            double height = heightMapCommand.getHeight(x, y);
            if (Double.isNaN(height) || !MathTools.intervalContains(height, minimumZToConsider, maximumZToConsider))
            {
               continue;
            }

            Point3D snappedPoint = pointsInsidePolygon.add();
            snappedPoint.set(x, y, height);
            maximumHeight = Math.max(maximumHeight, height);
         }
      }

      maxHeightYoVariable.set(maximumHeight);
      double heightThresholdToFilter = maximumHeight - maximumHeightDeltaToConsider.getDoubleValue();
      for (int i = 0; i < pointsInsidePolygon.size(); i++)
      {
         if (pointsInsidePolygon.get(i).getZ() > heightThresholdToFilter)
         {
            pointsInsidePolygonFiltered.add().set(pointsInsidePolygon.get(i));
         }
      }

      if (pointsInsidePolygonFiltered.size() < 3)
      {
         return true;
      }

      // Fit plane to points within footprint
      planeFitter.fitPlaneToPoints(pointsInsidePolygonFiltered, bestFitPlane);

      // Set position z of snapped foot
      double height = bestFitPlane.getZOnPlane(footstepPose.getX(), footstepPose.getY());
      adjustedPoseToPack.getLocation().getPoint().setZ(height);
      heightOfNewPose.set(height);

      // Set orientation of snapped foot
      if (includePitchAndRoll.getValue())
      {
         constructRotationToMatchSurfaceNormal(bestFitPlane.getNormal(), rotationMatrix);
         adjustedPoseToPack.getOrientation().getQuaternion().set(rotationMatrix);
         adjustedPoseToPack.getOrientation().getQuaternion().setToYawOrientation(footstepPose.getYaw());
      }

      return true;
   }

   public void setHeightMap(HeightMapCommand heightMapCommand)
   {
      this.heightMapCommand = heightMapCommand;
   }

   public void setSnapToHeightMap(boolean snapToHeightMap)
   {
      this.snapToHeightMap.set(snapToHeightMap);
   }
}
