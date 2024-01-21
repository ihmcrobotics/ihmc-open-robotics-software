package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
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
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator.interpolate;
import static us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator.scaleAdd;

public class SwingKnotPoint
{
   private static final double collisionBoxHeight = 0.4;
   private static final FramePose3D nanPose = new FramePose3D();

   static
   {
      nanPose.setToNaN();
   }

   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final double percentage;

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

   public SwingKnotPoint(int index,
                         double percentage,
                         SwingPlannerParametersReadOnly swingPlannerParameters,
                         WalkingControllerParameters walkingControllerParameters,
                         YoGraphicsListRegistry graphicsListRegistry,
                         YoRegistry registry)
   {
      this.swingPlannerParameters = swingPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.percentage = percentage;

      startingWaypoint = new YoFramePose3D("waypoint_init" + index, ReferenceFrame.getWorldFrame(), registry);
      optimizedWaypoint = new YoFramePose3D("waypoint_opt" + index, ReferenceFrame.getWorldFrame(), registry);
      waypointDisplacement = new YoDouble("waypoint_disp_" + index, registry);

      maxDisplacement = new YoDouble("maxDisplacement" + index, registry);
      maxDisplacementSquared = new YoDouble("maxDisplacementSq" + index, registry);

      waypointPoseFrame = new PoseReferenceFrame("waypointPoseFrame" + index, ReferenceFrame.getWorldFrame());
      collisionBoxPose = new YoFramePoseUsingYawPitchRoll("collisionBoxPose" + index, ReferenceFrame.getWorldFrame(), registry);

      waypointAdjustmentFrame = new PoseReferenceFrame("waypointAdjustmentFrame" + index, ReferenceFrame.getWorldFrame());
      waypointAdjustmentPose = new YoFramePose3D("waypointAdjustmentPose" + index, ReferenceFrame.getWorldFrame(), registry);

      // go ahead and initialize so the log viewer renders the correct dimensions
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
         AppearanceDefinition collisionBoxColor = YoAppearance.RGBColorFromHex(0x824e38);
         collisionBoxColor.setTransparency(0.8);
         collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
         yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic" + index, collisionBoxGraphic, collisionBoxPose, 1.0);
         graphicsListRegistry.registerYoGraphic(boxListName, yoCollisionBoxGraphic);

         adjustmentGraphic = new YoGraphicCoordinateSystem("waypointAdjGraphic" + index, waypointAdjustmentPose, 0.1);
         graphicsListRegistry.registerYoGraphic(waypointFrameListName, adjustmentGraphic);

         YoGraphicPosition waypointPositionGraphic = new YoGraphicPosition("waypointGraphic" + index, optimizedWaypoint.getPosition(), 0.01, YoAppearance.Black());
         graphicsListRegistry.registerYoGraphic(waypointListName, waypointPositionGraphic);
      }
   }

   // call once per footstep plan
   public void initializeBoxParameters()
   {
      initializeBoxParameters(walkingControllerParameters, swingPlannerParameters, percentage, collisionBox, boxCenterInSoleFrame);
   }

   public static void initializeBoxParameters(WalkingControllerParameters walkingControllerParameters,
                                              SwingPlannerParametersReadOnly swingPlannerParameters,
                                              double percentage,
                                              FrameBox3DBasics collisionBoxToPack,
                                              Vector3DBasics boxCenterInSoleFrameToPack)
   {
      // set default size
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double boxSizeX = footForwardOffset + footBackwardOffset;
      double boxSizeY = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double boxSizeZ = collisionBoxHeight;
      collisionBoxToPack.getSize().set(boxSizeX, boxSizeY, boxSizeZ);

      // add extra size
      for (int i = 0; i < 3; i++)
      {
         double extraSize = computeExtraSize(swingPlannerParameters, percentage, i);
         collisionBoxToPack.getSize().setElement(i, collisionBoxToPack.getSize().getElement(i) + extraSize);
      }

      double boxBaseInSoleFrameX = 0.5 * (footForwardOffset - footBackwardOffset);
      boxCenterInSoleFrameToPack.set(boxBaseInSoleFrameX, 0.0, 0.5 * collisionBoxHeight);
   }

   private final Vector3D adjustmentFrameX = new Vector3D();
   private final Vector3D adjustmentFrameY = new Vector3D();
   private final Vector3D adjustmentFrameZ = new Vector3D();

   public void initializeWaypointAdjustmentFrame(Vector3DReadOnly waypointVelocity, FramePoint3DReadOnly startOfSwing, FramePoint3DReadOnly endOfSwing)
   {
      double swingDx = endOfSwing.getX() - startOfSwing.getX();
      double swingDy = endOfSwing.getY() - startOfSwing.getY();
      Vector3D swingPlaneNormal = new Vector3D(-swingDy, swingDx, 0.0);

      adjustmentFrameX.set(waypointVelocity);
      adjustmentFrameX.normalize();

      adjustmentFrameY.cross(Axis3D.Z, adjustmentFrameX);
      adjustmentFrameY.normalize();
      if (swingPlaneNormal.dot(adjustmentFrameY) < 0.0)
         adjustmentFrameY.negate();

      adjustmentFrameZ.cross(adjustmentFrameX, adjustmentFrameY);
      waypointAdjustmentPose.getPosition().set(startingWaypoint.getPosition());

      double yaw = Math.atan2(adjustmentFrameX.getY(), adjustmentFrameX.getX());
      double pitch = -Math.asin(adjustmentFrameX.getZ());
      double roll = 0.0;
      waypointAdjustmentPose.getOrientation().setYawPitchRoll(yaw, pitch, roll);
      waypointAdjustmentFrame.setPoseAndUpdate(waypointAdjustmentPose);
   }

   // projects onto the YZ plane of the adjustment frame
   public void project(Vector3DBasics shiftDirection)
   {
      if (swingPlannerParameters.getAllowLateralMotion())
      {
         double xAlpha = adjustmentFrameX.dot(shiftDirection);
         scaleAdd(shiftDirection, -xAlpha, adjustmentFrameX);
      }
      else
      {
         double zAlpha = adjustmentFrameZ.dot(shiftDirection);
         shiftDirection.set(adjustmentFrameZ);
         shiftDirection.scale(zAlpha);
      }
   }

   public YoFramePose3D getStartingWaypoint()
   {
      return startingWaypoint;
   }

   public void initialize(FramePose3DReadOnly startingWaypoint)
   {
      this.startingWaypoint.set(startingWaypoint);
      waypointDisplacement.set(0.0);
      optimizedWaypoint.set(startingWaypoint);
      updateCollisionBox();
   }

   public void setMaxDisplacement(double maxDisplacement)
   {
      this.maxDisplacement.set(maxDisplacement);
      this.maxDisplacementSquared.set(maxDisplacement * maxDisplacement);
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
      if (scale < 1e-8)
      {
         return;
      }

      optimizedWaypoint.getPosition().set(tempPose.getPosition());
      waypointDisplacement.set(startingWaypoint.getPosition().distance(optimizedWaypoint.getPosition()));
      updateCollisionBox();
   }

   public double computeMaximumDisplacementScale(Vector3DReadOnly displacement)
   {
      double adjustmentScale = 1.0;

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
      boxCenterPose.setToZero(waypointPoseFrame);
      boxCenterPose.getPosition().set(boxCenterInSoleFrame);
      boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());
      collisionBox.getPose().set(boxCenterPose);
   }

   public double getDisplacement()
   {
      return waypointDisplacement.getDoubleValue();
   }

   public double getPercentage()
   {
      return percentage;
   }

   public boolean doCollisionCheck(ExpandingPolytopeAlgorithm collisionDetector, PlanarRegionsList planarRegionsList, HeightMapData heightMapData,
                                   SwingKnotOptimizationResult knotResult)
   {
      this.collisionResult.setToZero();
      this.collisionResult.setSignedDistance(Double.POSITIVE_INFINITY);

      if (planarRegionsList != null)
      {
         for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(collisionBox.getBoundingBox()))
            {
               EuclidShape3DCollisionResult collisionResult = collisionDetector.evaluateCollision(collisionBox, planarRegion);

               if (collisionResult.getSignedDistance() < this.collisionResult.getSignedDistance())
               {
                  this.collisionResult.set(collisionResult);
               }
            }
         }
      }
      if (heightMapData != null && !heightMapData.isEmpty())
      {
         EuclidShape3DCollisionResult collisionResult = HeightMapCollisionDetector.evaluateCollision(collisionBox, heightMapData);

         if (collisionResult.getSignedDistance() < this.collisionResult.getSignedDistance())
         {
            this.collisionResult.set(collisionResult);
         }
      }

      return collisionResult.areShapesColliding();
   }

   public EuclidShape3DCollisionResult getCollisionResult()
   {
      return collisionResult;
   }

   /**
    * Extruding the collision box outward is tricky since it can easily collide with the ground at the start/end.
    * To avoid this amount to expand is smoothly interpolated.
    */
   private double computeExtraSize(double percentage, int axisIndex)
   {
      return computeExtraSize(swingPlannerParameters, percentage, axisIndex);
   }

   /**
    * Extruding the collision box outward is tricky since it can easily collide with the ground at the start/end.
    * To avoid this amount to expand is smoothly interpolated.
    */
   private static double computeExtraSize(SwingPlannerParametersReadOnly swingPlannerParameters, double percentage, int axisIndex)
   {
      Axis3D axis = Axis3D.values()[axisIndex];
      double percentageLow = swingPlannerParameters.getExtraSizePercentageLow(axis);
      double percentageHigh = swingPlannerParameters.getExtraSizePercentageHigh(axis);
      double extraSizeLow = swingPlannerParameters.getExtraSizeLow(axis);
      double extraSizeHigh = swingPlannerParameters.getExtraSizeHigh(axis);
      return interpolate(percentage, percentageLow, percentageHigh, extraSizeLow, extraSizeHigh);
   }

   public void updateGraphics(boolean showBox)
   {
      if (showBox)
         yoCollisionBoxGraphic.setPose(boxCenterPose);
      else
         yoCollisionBoxGraphic.setPoseToNaN();
   }

   public void hide()
   {
      adjustmentGraphic.setPose(nanPose);
      optimizedWaypoint.setToNaN();
   }
}

