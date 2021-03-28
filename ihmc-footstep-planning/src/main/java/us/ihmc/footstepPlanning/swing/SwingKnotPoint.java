package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingKnotPoint
{
   private final SwingPlannerParametersReadOnly swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final double percentage;

   private final YoFramePose3D waypoint;
   private final YoFramePoseUsingYawPitchRoll collisionBoxPose;

   private final PoseReferenceFrame waypointPoseFrame;
   private final Vector3D boxCenterInSoleFrame = new Vector3D();
   private final FramePose3D boxCenterPose = new FramePose3D();
   private final FrameBox3D collisionBox = new FrameBox3D();

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

      waypoint = new YoFramePose3D("waypoint" + index, ReferenceFrame.getWorldFrame(), registry);
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

         YoGraphicPosition waypointPositionGraphic = new YoGraphicPosition("waypointGraphic" + index, waypoint.getPosition(), 0.03, YoAppearance.Black());
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

   public YoFramePose3D getWaypoint()
   {
      return waypoint;
   }

   public double getPercentage()
   {
      return percentage;
   }

   // updates box from waypoint pose
   public void update()
   {
      waypointPoseFrame.setPoseAndUpdate(waypoint);

      boxCenterPose.setToZero(waypointPoseFrame);
      boxCenterPose.getPosition().set(boxCenterInSoleFrame);
      boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());
      collisionBox.getPose().set(boxCenterPose);
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

