package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

import java.awt.*;
import java.util.List;

public class EnvironmentConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double distanceInsideRegion = 0.06;

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoShrunkConvexHullConstraint;

   private PlanarRegion planarRegionToConstrainTo = null;

   private final ConvexPolygon2D footstepPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform footOrientationTransform = new RigidBodyTransform();
   private final FramePoint2D stepXY = new FramePoint2D();

   public EnvironmentConstraintHandler(SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                       String yoNamePrefix,
                                       YoVariableRegistry registry,
                                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactableFeet = contactableFeet;

      yoConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraint", "", worldFrame, 12, registry);
      yoShrunkConvexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ShrunkConvexHullConstraint", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         YoArtifactPolygon shrunkActivePlanarRegionViz = new YoArtifactPolygon("ShrunkConvexHullConstraint",
                                                                               yoShrunkConvexHullConstraint,
                                                                               Color.RED,
                                                                               false,
                                                                               true);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), shrunkActivePlanarRegionViz);
      }
   }

   public void setPlanarRegionConstraint(PlanarRegion planarRegionToConstrainTo)
   {
      this.planarRegionToConstrainTo = planarRegionToConstrainTo;
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      yoConvexHullConstraint.clear();
      yoShrunkConvexHullConstraint.clear();
   }

   public void applyEnvironmentConstraintToFootstep(RobotSide upcomingFootstepSide,
                                                    FixedFramePose3DBasics footstepPoseToPack,
                                                    List<Point2D> predictedContactPoints)
   {
      if (planarRegionToConstrainTo == null)
         return;

      computeShrunkConvexHull(planarRegionToConstrainTo, upcomingFootstepSide, predictedContactPoints, footstepPoseToPack.getOrientation());

      stepXY.set(footstepPoseToPack.getPosition());
      yoShrunkConvexHullConstraint.orthogonalProjection(stepXY);

      footstepPoseToPack.getPosition().set(stepXY, planarRegionToConstrainTo.getPlaneZGivenXY(stepXY.getX(), stepXY.getY()));
      footstepPoseToPack.getOrientation().set(planarRegionToConstrainTo.getTransformToWorld().getRotation());
   }


   private void computeShrunkConvexHull(PlanarRegion planarRegion,
                                        RobotSide upcomingFootstepSide,
                                        List<? extends Point2DBasics> predictedContactPoints,
                                        Orientation3DReadOnly orientation)
   {
      computeFootstepPolygon(upcomingFootstepSide, predictedContactPoints, orientation);

      yoConvexHullConstraint.set(planarRegion.getConvexHull());
      yoConvexHullConstraint.applyTransform(planarRegion.getTransformToWorld(), false);

      scaler.scaleConvexPolygonToContainInteriorPolygon(yoConvexHullConstraint, footstepPolygon, distanceInsideRegion, yoShrunkConvexHullConstraint);
   }

   private void computeFootstepPolygon(RobotSide upcomingFootstepSide, List<? extends Point2DBasics> predictedContactPoints, Orientation3DReadOnly orientation)
   {
      if (predictedContactPoints.isEmpty())
         predictedContactPoints = contactableFeet.get(upcomingFootstepSide).getContactPoints2d();

      footstepPolygon.clear();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         footstepPolygon.addVertex(predictedContactPoints.get(i));
      footstepPolygon.update();

      footOrientationTransform.getRotation().set(orientation);

      footstepPolygon.applyTransform(footOrientationTransform);
   }
}
