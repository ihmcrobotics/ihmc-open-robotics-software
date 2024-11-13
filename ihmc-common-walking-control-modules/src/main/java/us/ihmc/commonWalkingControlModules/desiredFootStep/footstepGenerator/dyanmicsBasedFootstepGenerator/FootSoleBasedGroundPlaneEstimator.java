package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.geometry.YoGroundPlaneEstimator;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class FootSoleBasedGroundPlaneEstimator
{
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final MovingReferenceFrame centerOfMassControlZUpFrame;

   private final YoGroundPlaneEstimator groundPlaneEstimator;
   private final List<Point3DReadOnly> contactPoints = new ArrayList<>();
   private final RecyclingArrayList<FramePoint3D> frameContactPoints = new RecyclingArrayList<>(FramePoint3D.class);
   private RobotSide supportSide = RobotSide.LEFT;

   private final FramePoint3D groundPlanePoint;

   private final YoDouble groundPlaneVerticalOffset;
   private final YoBoolean accountForGroundOrientation;


   private final ZUpFrame groundPlaneZUpFrame;

   /**
    * Determines the position and orientation of the steppable region either by assuming flat ground,
    * in which case the location and orientation of the ground matches that of the stance foot, or by
    * using planar regions from a {@code PlanarRegionsListCommand}
    *
    * @param parentRegistry the {@code YoRegistry} used to register YoVariables constructed within this class
    */
   public FootSoleBasedGroundPlaneEstimator(MovingReferenceFrame centerOfMassControlZUpFrame, CommonHumanoidReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.centerOfMassControlZUpFrame = centerOfMassControlZUpFrame;

      this.referenceFrames = referenceFrames;

      groundPlaneEstimator = new YoGroundPlaneEstimator("FootSoleBased",
                                                        parentRegistry,
                                                        yoGraphicsListRegistry,
                                                        YoAppearance.Red());

      groundPlaneVerticalOffset = new YoDouble("groundPlaneVerticalOffset", parentRegistry);
      accountForGroundOrientation = new YoBoolean("accountForGroundOrientation", parentRegistry);

      groundPlanePoint = new FramePoint3D();

      groundPlaneZUpFrame = new ZUpFrame(getGroundPlaneFrame(), "groundPlaneZUpFrame");

      contactPoints.add(new Point3D(-0.1, -0.1, 0.0));
      contactPoints.add(new Point3D(-0.1, 0.1, 0.0));
      contactPoints.add(new Point3D(0.1, 0.1, 0.0));
      contactPoints.add(new Point3D(0.1, -0.1, 0.0));

      for (int i = 0; i < contactPoints.size(); i++)
         frameContactPoints.add();
   }

   public void reset()
   {
      // TODO detect the sides that are on the ground, in case we call reset on entering the state where one foot isn't on the ground, or the support side is
      // wrong
//      supportSide = controllerToolbox.getCommonVariables().getSwingside().getEnumValue().getOppositeSide();

      updateFrameContactPoints(false);
      computeGroundPlaneEstimator();
   }

   public void update()
   {
      groundPlaneZUpFrame.update();

      update(false, false, false);
   }

   public void update(boolean isToeingOff, boolean isFootSlipping, boolean isFootRocking)
   {
      groundPlaneZUpFrame.update();

      if (!isFootSlipping && !isFootRocking)
      {
         updateFrameContactPoints(isToeingOff);
         computeGroundPlaneEstimator();
      }
   }

   private void updateFrameContactPoints(boolean isInToeOff)
   {
      FramePoint3D lowestContactPoint = frameContactPoints.get(0);

      for (int i = 0; i < frameContactPoints.size(); i++)
      {
         FramePoint3D frameContactPoint = frameContactPoints.get(i);

         if (!isInToeOff)
         {
            frameContactPoint.setMatchingFrame(referenceFrames.getSoleFrame(supportSide), contactPoints.get(i));
            frameContactPoint.addZ(groundPlaneVerticalOffset.getDoubleValue());
         }
         else if (i > 1)
         {
//            frameContactPoint.setMatchingFrame(controllerToolbox.getEstimates().getToePosition(supportSide));
            frameContactPoint.addZ(groundPlaneVerticalOffset.getDoubleValue());
         }

         if (frameContactPoint.getZ() < lowestContactPoint.getZ())
            lowestContactPoint = frameContactPoint;
      }

      for (int i = 0; i < frameContactPoints.size(); i++)
      {
         FramePoint3D frameContactPoint = frameContactPoints.get(i);

         if (!accountForGroundOrientation.getBooleanValue())
            frameContactPoint.setZ(lowestContactPoint.getZ());
      }
   }

   private void computeGroundPlaneEstimator()
   {
      double yaw = centerOfMassControlZUpFrame.getTransformToWorldFrame().getRotation().getYaw();

      groundPlaneEstimator.compute(frameContactPoints, yaw);
      groundPlaneEstimator.getPlanePoint(groundPlanePoint);
   }

   public FrameTuple3DReadOnly getGroundPosition()
   {
      return groundPlanePoint;
   }

   public FrameTuple3DReadOnly getGroundPosition(FrameTuple2DReadOnly touchDownPosition2D)
   {
      getGroundPosition(groundPlanePoint, touchDownPosition2D, groundPlaneEstimator.getPlane());
      return groundPlanePoint;
   }

   public static void getGroundPosition(FramePoint3DBasics groundPosition3DToPack, FrameTuple2DReadOnly touchDownPosition2D, Plane3DReadOnly groundPlane)
   {
      groundPosition3DToPack.setMatchingFrame(touchDownPosition2D, 0.0);
      GroundPlaneEstimator.projectZ(groundPosition3DToPack, groundPlane);
   }

   public FrameOrientation3DReadOnly getGroundOrientation()
   {
      return (FrameOrientation3DReadOnly) groundPlaneEstimator.getGroundPlaneFrame().getOrientation();
   }

   public Plane3DReadOnly getGroundPlane()
   {
      return groundPlaneEstimator.getPlane();
   }

   public ReferenceFrame getGroundPlaneFrame()
   {
      return groundPlaneEstimator.getGroundPlaneFrame();
   }

   public ReferenceFrame getGroundPlaneZUpFrame()
   {
      return groundPlaneZUpFrame;
   }

   public void hideGraphics()
   {
      groundPlaneEstimator.hideGraphics();
   }
}
