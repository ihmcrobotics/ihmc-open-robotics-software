package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.statistics.Line2DStatisticsCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.awt.Color;

public class GeometricRotationEdgeCalculator implements RotationEdgeCalculator
{
   private final YoVariableRegistry registry;

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MovingReferenceFrame soleFrame;

   private final FrameVector3D lineOfContact = new FrameVector3D();
   private final FrameVector3D footNormal = new FrameVector3D();
   private final YoFrameVector3D groundPlaneNormal;

   private final YoFramePoint2D linePointA;
   private final YoFramePoint2D linePointB;

   private final FrameLine2D lineOfRotationInWorldFrame = new FrameLine2D();

   private final YoFrameLine2D lineOfRotationInSole;

   private final Line2DStatisticsCalculator lineOfRotationStandardDeviation;

   public GeometricRotationEdgeCalculator(RobotSide side, MovingReferenceFrame soleFrame, YoVariableRegistry parentRegistry,
                                          YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrame = soleFrame;

      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      linePointA = new YoFramePoint2D("FootRotationPointA", ReferenceFrame.getWorldFrame(), registry);
      linePointB = new YoFramePoint2D("FootRotationPointB", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoint2D point = new YoFramePoint2D(side.getLowerCaseName() + "LineOfRotationPoint", soleFrame, registry);
      YoFrameVector2D direction = new YoFrameVector2D(side.getLowerCaseName() + "LineOfRotationDirection", soleFrame, registry);
      lineOfRotationInSole = new YoFrameLine2D(point, direction);

      groundPlaneNormal = new YoFrameVector3D(side.getShortLowerCaseName() + "PlaneNormal", worldFrame, registry);

      lineOfRotationStandardDeviation = new Line2DStatisticsCalculator(side.getLowerCaseName() + "LineOfRotation", lineOfRotationInSole, registry);

      parentRegistry.addChild(registry);

      reset();

      if (graphicsListRegistry != null)
      {
         Artifact lineArtifact = new YoArtifactLineSegment2d(side.getLowerCaseName() + "LineOfRotation", linePointA, linePointB, Color.ORANGE, 0.005, 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineArtifact);
      }
   }

   private final FramePoint3D tempPointOfRotation = new FramePoint3D();

   public void compute(FramePoint2DReadOnly measuredCoP)
   {
      groundPlaneNormal.set(worldFrame, 0.0, 0.0, 1.0);

      // intersect the foot plane and the ground plane
      footNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0);
      footNormal.normalize();
      footNormal.changeFrame(worldFrame);
      lineOfContact.cross(groundPlaneNormal, footNormal);

      tempPointOfRotation.setIncludingFrame(measuredCoP, 0.0);
      lineOfRotationInWorldFrame.setToZero(worldFrame);
      lineOfRotationInWorldFrame.set(tempPointOfRotation, lineOfContact);
      lineOfRotationInSole.setMatchingFrame(lineOfRotationInWorldFrame);

      updateGraphics();
   }

   public void reset()
   {
      linePointA.setToNaN();
      linePointB.setToNaN();

      lineOfRotationInSole.setToZero();

      lineOfRotationStandardDeviation.reset();
   }

   private void updateGraphics()
   {
      linePointA.set(lineOfRotationInWorldFrame.getDirection());
      linePointA.scale(-0.05);
      linePointA.add(lineOfRotationInWorldFrame.getPoint());

      linePointB.set(lineOfRotationInWorldFrame.getDirection());
      linePointB.scale(0.05);
      linePointB.add(lineOfRotationInWorldFrame.getPoint());
   }

   public FrameLine2DReadOnly getLineOfRotation()
   {
      return lineOfRotationInSole;
   }
}
