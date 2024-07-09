package us.ihmc.footstepPlanning.narrowPassage;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;

public class BodyCollisionPoint
{
   // Scale applied to position shift, used to converge on center of oscillation
   private static double shiftAlpha = 1;

   private final DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final YoFramePose3D startingWaypoint;
   private final YoFramePose3D optimizedWaypoint;
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose;

   private final PoseReferenceFrame waypointPoseFrame;
   private final Vector3D boxCenterInSoleFrame = new Vector3D();
   private final FramePose3D boxCenterPose = new FramePose3D();
   private final FrameBox3D collisionBox = new FrameBox3D();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

   private final PoseReferenceFrame waypointAdjustmentFrame;
   private final YoFramePose3D waypointAdjustmentPose;
   private final YoGraphicShape yoCollisionBoxGraphic;
   private final YoGraphicCoordinateSystem adjustmentGraphic;

   public BodyCollisionPoint(int index,
                             DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters,
                             YoGraphicsListRegistry graphicsListRegistry,
                             YoRegistry registry)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      startingWaypoint = new YoFramePose3D("waypoint_init" + index, ReferenceFrame.getWorldFrame(), registry);
      optimizedWaypoint = new YoFramePose3D("waypoint_opt" + index, ReferenceFrame.getWorldFrame(), registry);

      waypointPoseFrame = new PoseReferenceFrame("waypointPoseFrame" + index, ReferenceFrame.getWorldFrame());
      collisionBoxPose = new YoFramePoseUsingYawPitchRoll("collisionBoxPose" + index, ReferenceFrame.getWorldFrame(), registry);

      waypointAdjustmentFrame = new PoseReferenceFrame("waypointAdjustmentFrame" + index, ReferenceFrame.getWorldFrame());
      waypointAdjustmentPose = new YoFramePose3D("waypointAdjustmentPose" + index, ReferenceFrame.getWorldFrame(), registry);

      initializeBoxParameters();

      if (graphicsListRegistry == null)
      {
         yoCollisionBoxGraphic = null;
         adjustmentGraphic = null;
      }
      else
      {
         String boxListName = "boxes";
         String waypointFrameListName = "frames";
         String waypointListName = "waypoints";

         Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
         AppearanceDefinition collisionBoxColor = YoAppearance.Color(Color.GREEN);
         collisionBoxColor.setTransparency(0.9);
         collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
         yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic" + index, collisionBoxGraphic, collisionBoxPose, 1.0);
         graphicsListRegistry.registerYoGraphic(boxListName, yoCollisionBoxGraphic);

         adjustmentGraphic = new YoGraphicCoordinateSystem("waypointAdjGraphic" + index, waypointAdjustmentPose, 0.1);
         graphicsListRegistry.registerYoGraphic(waypointFrameListName, adjustmentGraphic);

         YoGraphicPosition waypointPositionGraphic = new YoGraphicPosition("waypointGraphic" + index,
                                                                           optimizedWaypoint.getPosition(),
                                                                           0.01,
                                                                           YoAppearance.Black());
         graphicsListRegistry.registerYoGraphic(waypointListName, waypointPositionGraphic);
         updateGraphics(false);
      }
   }

   public void initializeBoxParameters()
   {
      double boxSizeX = footstepPlannerParameters.getBodyBoxDepth();
      double boxSizeY = footstepPlannerParameters.getBodyBoxWidth();
      double boxSizeZ = footstepPlannerParameters.getBodyBoxHeight();

      collisionBox.getSize().set(boxSizeX, boxSizeY, boxSizeZ);
      boxCenterInSoleFrame.set(footstepPlannerParameters.getBodyBoxBaseX(), 0.0, boxSizeZ / 2.0 + footstepPlannerParameters.getBodyBoxBaseZ());
   }

   private final Vector3D adjustmentFrameX = new Vector3D();
   private final Vector3D adjustmentFrameY = new Vector3D();
   private final Vector3D adjustmentFrameZ = new Vector3D();

   public void initializeWaypointAdjustmentFrame(Vector3DReadOnly nextWaypointDirection)
   {
      adjustmentFrameX.set(nextWaypointDirection);
      adjustmentFrameX.normalize();

      adjustmentFrameY.cross(Axis3D.Z, adjustmentFrameX);
      adjustmentFrameY.normalize();

      adjustmentFrameZ.cross(adjustmentFrameX, adjustmentFrameY);
      waypointAdjustmentPose.getPosition().set(startingWaypoint.getPosition());

      double yaw = Math.atan2(adjustmentFrameX.getY(), adjustmentFrameX.getX());
      double pitch = 0.0;
      double roll = 0.0;
      waypointAdjustmentPose.getOrientation().setYawPitchRoll(yaw, pitch, roll);
      waypointAdjustmentFrame.setPoseAndUpdate(waypointAdjustmentPose);
   }

   public void project(Vector3DBasics shiftDirection)
   {
      double yAlpha = adjustmentFrameY.dot(shiftDirection);
      shiftDirection.set(adjustmentFrameY);
      shiftDirection.scale(yAlpha * 0.5);
   }

   public int project(Vector3DBasics shiftDirection, int previousShiftSign)
   {
      double yAlpha = adjustmentFrameY.dot(shiftDirection);
      shiftDirection.set(adjustmentFrameY);
      if ((previousShiftSign > 0 && yAlpha < 0) || (previousShiftSign < 0 && yAlpha > 0))
         shiftAlpha = shiftAlpha * 0.85;
      shiftDirection.scale(yAlpha * shiftAlpha);
      return yAlpha > 0 ? 1 : -1;
   }

   public static void resetShiftAlpha()
   {
      shiftAlpha = 1;
   }

   public void initialize(FramePose3DReadOnly startingWaypoint)
   {
      this.startingWaypoint.set(startingWaypoint);
      optimizedWaypoint.set(startingWaypoint);
      optimizedWaypoint.getOrientation().set(waypointAdjustmentPose.getOrientation());
      updateCollisionBox();
   }

   public YoFramePose3D getOptimizedWaypoint()
   {
      return optimizedWaypoint;
   }

   public void shiftWaypoint(Vector3DReadOnly displacement)
   {
      optimizedWaypoint.getPosition().add(displacement);
      updateCollisionBox();
   }

   public double getYawShift(Vector3D collisionVector)
   {
      // Scale turning angle based on collision vector
      double yawShift = Math.toRadians(collisionVector.length() * 250);
      if (yawShift < Math.toRadians(1))
         yawShift = Math.toRadians(1);
      return yawShift;
   }

   public void adjustOrientation(int rotationDirection, double yawShift)
   {
      QuaternionReadOnly orientation = optimizedWaypoint.getOrientation();
      optimizedWaypoint.getOrientation().setYawPitchRoll(orientation.getYaw() + rotationDirection * yawShift, orientation.getPitch(), orientation.getRoll());
      updateCollisionBox();
   }

   public int getRotationDirection(Vector3D collisionVector)
   {
      double yAlpha = adjustmentFrameY.dot(collisionVector);
      return yAlpha < 0 ? 1 : -1;
   }

   private void updateCollisionBox()
   {
      waypointPoseFrame.setPoseAndUpdate(optimizedWaypoint);
      boxCenterPose.setToZero(waypointPoseFrame);
      boxCenterPose.getPosition().set(boxCenterInSoleFrame);
      boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());
      collisionBox.getPose().set(boxCenterPose);
      collisionBox.getOrientation().set(optimizedWaypoint.getOrientation());
   }

   public boolean doCollisionCheck(ExpandingPolytopeAlgorithm collisionDetector, PlanarRegionsList planarRegionsList)
   {
      this.collisionResult.setToZero();
      this.collisionResult.setSignedDistance(Double.POSITIVE_INFINITY);

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(collisionBox.getBoundingBox()))
         {
            EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
            collisionDetector.evaluateCollision(collisionBox, planarRegion, collisionResult);

            if (collisionResult.getSignedDistance() < this.collisionResult.getSignedDistance())
            {
               this.collisionResult.set(collisionResult);
            }
         }
      }

      return collisionResult.areShapesColliding();
   }

   public EuclidShape3DCollisionResult getCollisionResult()
   {
      return collisionResult;
   }

   public void updateGraphics(boolean showBox)
   {
      adjustmentGraphic.setPosition(optimizedWaypoint.getPosition());
      adjustmentGraphic.setOrientation(optimizedWaypoint.getOrientation());

      if (showBox)
      {
         yoCollisionBoxGraphic.setPose(boxCenterPose);
         yoCollisionBoxGraphic.setOrientation(optimizedWaypoint.getOrientation());
      }
      else
         yoCollisionBoxGraphic.setPoseToNaN();
   }

   public void hide()
   {
      updateGraphics(false);
      adjustmentGraphic.setPose(new FramePose3D());
      optimizedWaypoint.setToNaN();
   }
}
