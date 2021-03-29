package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SwingKnotPoint
{
   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final double percentage;

   private final YoFramePose3D startingWaypoint;
   private final YoFramePose3D currentWaypoint;
   private final YoDouble waypointDisplacement;
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose;
   private final YoDouble maxDisplacement;
   private final YoDouble maxDisplacementSquared;

   private final PoseReferenceFrame waypointPoseFrame;
   private final Vector3D boxCenterInSoleFrame = new Vector3D();
   private final FramePose3D boxCenterPose = new FramePose3D();
   private final FrameBox3D collisionBox = new FrameBox3D();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

   private final YoGraphicShape yoCollisionBoxGraphic;

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

      startingWaypoint = new YoFramePose3D("waypoint_s" + index, ReferenceFrame.getWorldFrame(), registry);
      currentWaypoint = new YoFramePose3D("waypoint_c" + index, ReferenceFrame.getWorldFrame(), registry);
      waypointDisplacement = new YoDouble("waypoint_disp_" + index, registry);

      maxDisplacement = new YoDouble("maxDisplacement" + index, registry);
      maxDisplacementSquared = new YoDouble("maxDisplacementSq" + index, registry);

      waypointPoseFrame = new PoseReferenceFrame("waypointPoseFrame" + index, ReferenceFrame.getWorldFrame());
      collisionBoxPose = new YoFramePoseUsingYawPitchRoll("collisionBoxPose" + index, ReferenceFrame.getWorldFrame(), registry);

      // go ahead and initialize so the log viewer has the correct dimensions
      initializeBoxParameters();

      if (graphicsListRegistry == null)
      {
         yoCollisionBoxGraphic = null;
      }
      else
      {
         Graphics3DObject collisionBoxGraphic = new Graphics3DObject();
         AppearanceDefinition collisionBoxColor = YoAppearance.RGBColorFromHex(0x824e38);
         collisionBoxColor.setTransparency(0.8);
         collisionBoxGraphic.addCube(collisionBox.getSizeX(), collisionBox.getSizeY(), collisionBox.getSizeZ(), true, collisionBoxColor);
         yoCollisionBoxGraphic = new YoGraphicShape("collisionGraphic" + index, collisionBoxGraphic, collisionBoxPose, 1.0);
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), yoCollisionBoxGraphic);

         YoGraphicPosition waypointPositionGraphic = new YoGraphicPosition("waypointGraphic" + index, currentWaypoint.getPosition(), 0.03, YoAppearance.Black());
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), waypointPositionGraphic);
      }
   }

   // call once per footstep plan
   public void initializeBoxParameters()
   {
      double minPercentageToCheck = swingPlannerParameters.getMinMaxCheckerPercentage();
      double maxPercentageToCheck = 1.0 - swingPlannerParameters.getMinMaxCheckerPercentage();
      double alphaExtraHeight = computeAlphaExtraHeight(percentage, minPercentageToCheck, maxPercentageToCheck);
      double collisionBoxExtraZ = alphaExtraHeight * swingPlannerParameters.getCollisionBoxExtraZ();

      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double boxSizeX = footForwardOffset + footBackwardOffset + 2.0 * swingPlannerParameters.getCollisionBoxExtraX();
      double boxSizeY = walkingControllerParameters.getSteppingParameters().getFootWidth() + 2.0 * swingPlannerParameters.getCollisionBoxExtraY();
      double boxSizeZ = swingPlannerParameters.getCollisionBoxHeight() + 2.0 * collisionBoxExtraZ;
      collisionBox.getSize().set(boxSizeX, boxSizeY, boxSizeZ);

      double boxBaseInSoleFrameX = 0.5 * (footForwardOffset - footBackwardOffset);
      boxCenterInSoleFrame.set(boxBaseInSoleFrameX, 0.0, 0.5 * swingPlannerParameters.getCollisionBoxHeight());
   }

   public YoFramePose3D getStartingWaypoint()
   {
      return startingWaypoint;
   }

   public void initialize(FramePose3DReadOnly startingWaypoint)
   {
      waypointDisplacement.set(0.0);
      this.startingWaypoint.set(startingWaypoint);
      this.currentWaypoint.set(startingWaypoint);
      updateCollisionBox();
   }

   public void setMaxDisplacement(double maxDisplacement)
   {
      this.maxDisplacement.set(maxDisplacement);
      this.maxDisplacementSquared.set(maxDisplacement * maxDisplacement);
   }

   public YoFramePose3D getCurrentWaypoint()
   {
      return currentWaypoint;
   }

   private final FramePose3D tempPose = new FramePose3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void shiftWaypoint(Vector3DReadOnly displacement)
   {
      double adjustmentScale = 1.0;

      // try max of 5 times to scale down and check if valid to shift
      int maxAttempts = 5;
      for (int i = 0; i < maxAttempts; i++)
      {
         tempPose.set(currentWaypoint);
         tempVector.set(displacement);
         tempVector.scale(adjustmentScale);
         tempPose.getPosition().add(tempVector);
         boolean validShift = tempPose.getPosition().distanceSquared(startingWaypoint.getPosition()) < maxDisplacementSquared.getDoubleValue();
         if (validShift)
            break;
         adjustmentScale *= 0.5;
         if (i == maxAttempts - 1)
            return;
      }

      currentWaypoint.getPosition().set(tempPose.getPosition());
      waypointDisplacement.set(startingWaypoint.getPosition().distance(currentWaypoint.getPosition()));
      updateCollisionBox();
   }

   private void updateCollisionBox()
   {
      waypointPoseFrame.setPoseAndUpdate(currentWaypoint);
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

   public boolean doCollisionCheck(ExpandingPolytopeAlgorithm collisionDetector, PlanarRegionsList planarRegionsList)
   {
      this.collisionResult.setToZero();

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(collisionBox.getBoundingBox()))
         {
            EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
            collisionDetector.evaluateCollision(collisionBox, planarRegion, collisionResult);
            if (collisionResult.areShapesColliding() && (!this.collisionResult.areShapesColliding()
                                                         || collisionResult.getDistance() > this.collisionResult.getDistance()))
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

   /**
    * Extruding the collision box vertically up/down is tricky since it can easily collide with the ground.
    * To avoid this the extrusion amount starts at 0.0, ramps up to a max value, then ramps back down.
    */
   private double computeAlphaExtraHeight(double percentage, double minPercentageToCheck, double maxPercentageToCheck)
   {
      double minPForMaxHeight = swingPlannerParameters.getMinMaxHeightInterpolationPercentage();
      double maxPForMaxHeight = 1.0 - swingPlannerParameters.getMinMaxHeightInterpolationPercentage();
      if (MathTools.intervalContains(percentage, minPForMaxHeight, maxPForMaxHeight))
      {
         return 1.0;
      }
      else if (percentage < 0.5)
      {
         return EuclidCoreTools.interpolate(0.0, 1.0, (percentage - minPercentageToCheck) / (minPForMaxHeight - minPercentageToCheck));
      }
      else
      {
         return EuclidCoreTools.interpolate(1.0, 0.0, (percentage - maxPForMaxHeight) / (maxPercentageToCheck - maxPForMaxHeight));
      }
   }

   public void updateGraphics()
   {
      yoCollisionBoxGraphic.setPose(boxCenterPose);
   }
}

