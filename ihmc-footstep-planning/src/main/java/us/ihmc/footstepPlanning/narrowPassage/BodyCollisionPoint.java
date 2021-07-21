package us.ihmc.footstepPlanning.narrowPassage;

import sun.rmi.runtime.Log;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;

import static us.ihmc.footstepPlanning.narrowPassage.NarrowPassageBodyPathPlanner.scaleAdd;

public class BodyCollisionPoint
{
   private static final double collisionBoxHeight = 0.4;
   private static final FramePose3D nanPose = new FramePose3D();

   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final YoFramePose3D startingWaypoint;
   private final YoFramePose3D optimizedWaypoint;
   private final YoDouble waypointDisplacement;
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose;
   private final YoDouble maxDisplacement;
   private final YoDouble maxDisplacementSquared;

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
                             FootstepPlannerParametersReadOnly footstepPlannerParameters,
                             YoGraphicsListRegistry graphicsListRegistry,
                             YoRegistry registry)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      startingWaypoint = new YoFramePose3D("waypoint_init" + index, ReferenceFrame.getWorldFrame(), registry);
      optimizedWaypoint = new YoFramePose3D("waypoint_opt" + index, ReferenceFrame.getWorldFrame(), registry);
      waypointDisplacement = new YoDouble("waypoint_disp_" + index, registry);

      maxDisplacement = new YoDouble("maxDisplacement" + index, registry);
      maxDisplacementSquared = new YoDouble("maxDisplacementSq" + index, registry);

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
      }
   }

   public void initializeBoxParameters()
   {
      // set default size
      double boxSizeX = 0.4;   //footstepPlannerParameters.getBodyBoxDepth();
      double boxSizeY = 0.9;  //footstepPlannerParameters.getBodyBoxWidth();
      double boxSizeZ = 1.5;   //footstepPlannerParameters.getBodyBoxHeight();
      collisionBox.getSize().set(boxSizeX, boxSizeY, boxSizeZ);
      boxCenterInSoleFrame.set(footstepPlannerParameters.getBodyBoxBaseX(), footstepPlannerParameters.getBodyBoxBaseY(), boxSizeZ / 2 + 0.1);
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
      double pitch = -Math.asin(adjustmentFrameX.getZ());
      double roll = 0.0;
      waypointAdjustmentPose.getOrientation().setYawPitchRoll(yaw, pitch, roll);
      waypointAdjustmentFrame.setPoseAndUpdate(waypointAdjustmentPose);
   }

   public void project(Vector3DBasics shiftDirection)
   {
      double yAlpha = adjustmentFrameY.dot(shiftDirection);
      shiftDirection.set(adjustmentFrameY);
      shiftDirection.scale(yAlpha);
   }

   public YoFramePose3D getStartingWaypoint()
   {
      return startingWaypoint;
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

   private final FramePose3D tempPose = new FramePose3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void shiftWaypoint(Vector3DReadOnly displacement)
   {
      double scale = computeMaximumDisplacementScale(displacement);
      //      if (scale < 1e-8)
      //      {
      //         return;
      //      }

      optimizedWaypoint.getPosition().set(tempPose.getPosition());
      waypointDisplacement.set(startingWaypoint.getPosition().distance(optimizedWaypoint.getPosition()));
      updateCollisionBox();
   }

   public void adjustOrientation(int rotationDirection, Vector3D collisionVector)
   {
      // Scale turning angle based on collision vector
      double scale = collisionVector.length() * 200;
      if (scale < 2)
         scale = 2;

      QuaternionReadOnly orientation = optimizedWaypoint.getOrientation();
      optimizedWaypoint.getOrientation()
                       .setYawPitchRoll(orientation.getYaw() + Math.toRadians(rotationDirection * scale), orientation.getPitch(), orientation.getRoll());
      updateCollisionBox();
   }

   public int getRotationDirection(Vector3D collisionVector)
   {
      double yAlpha = adjustmentFrameY.dot(collisionVector);
      return yAlpha < 0 ? 1 : -1;
   }

   public double computeMaximumDisplacementScale(Vector3DReadOnly displacement)
   {
      double adjustmentScale = 1.0;

      // TODO Find a better place to set this
      maxDisplacementSquared.set(0.01);

      // try max of 5 times to scale down and check if valid to shift
      int maxAttempts = 5;
      for (int i = 0; i < maxAttempts; i++)
      {
         tempPose.set(optimizedWaypoint);
         tempVector.set(displacement);
         tempVector.scale(adjustmentScale);
         tempPose.getPosition().add(tempVector);
         boolean validShift = tempPose.getPosition().distanceSquared(startingWaypoint.getPosition()) < maxDisplacementSquared.getDoubleValue();
         if (validShift)
            break;
         adjustmentScale *= 0.5;
         if (i == maxAttempts - 1)
         {
            return 0.0;
         }
      }

      return adjustmentScale;
   }

   private void updateCollisionBox()
   {
      waypointPoseFrame.setPoseAndUpdate(optimizedWaypoint);
//      LogTools.info("waypointPoseFrame: " + waypointPoseFrame);
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
      adjustmentGraphic.setPose(nanPose);
      optimizedWaypoint.setToNaN();
   }
}
