package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d.IntersectionResult;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

/**
 * This class can detect if the foot is on partial support.
 * The idea is to keep track of where the CoP has been and use those points to compute the ground plane. If
 * a point on the foot drops below that plane it can be assumed that there is no ground under that point.
 *
 * @author Georg
 *
 */
public class GeometricFootRotationCalculator implements FootRotationCalculator
{
   private final static boolean VISUALIZE = false;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private static final Vector3D zero = new Vector3D(0.0, 0.0, 0.0);
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d defaultFootPolygon;

   private final FramePoint3D groundPlanePoint = new FramePoint3D();
   private final FrameVector3D groundPlaneNormal = new FrameVector3D();
   private final FrameLine2D lineOfRotationInSoleFrame = new FrameLine2D();
   private final FrameLine2D lineOfRotationInWorldFrame = new FrameLine2D();
   private final FrameConvexPolygon2d footPolygonInWorld = new FrameConvexPolygon2d();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   private final FramePoint3D cop = new FramePoint3D();
   private final YoDouble copAlpha;
   private final AlphaFilteredYoFramePoint copFiltered;
   private final FrameLineSegment2D lineSegmentOfRotation = new FrameLineSegment2D();

   private final YoFrameLineSegment2d yoLineOfRotation;
   private final YoFramePoint yoPlanePoint;
   private final YoFrameVector yoPlaneNormal;

   private final YoDouble angleFootGround;
   private final YoDouble angleTreshold;
   private final YoBoolean footRotating;

   public GeometricFootRotationCalculator(String namePrefix,
         ContactablePlaneBody contactableFoot,
         ExplorationParameters explorationParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());

      angleFootGround = new YoDouble(namePrefix + "AngleToGround", registry);
      angleTreshold = explorationParameters.getGeometricDetectionAngleThreshold();
      footRotating = new YoBoolean(namePrefix + "RotatingGeometry", registry);

      copAlpha = explorationParameters.getGeometricDetectionPlanePointAlpha();
      copFiltered = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint(namePrefix + "CoPFiltered",
            "", registry, copAlpha, worldFrame);

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();
         String caption = "";

         caption = namePrefix + "PlanePoint";
         yoPlanePoint = new YoFramePoint(caption, worldFrame, registry);
         YoGraphicPosition planePointViz =
               new YoGraphicPosition(caption, yoPlanePoint, 0.005, YoAppearance.Blue(), GraphicType.SOLID_BALL);
         YoArtifactPosition planePointArtifact = planePointViz.createArtifact();
         planePointArtifact.setVisible(VISUALIZE);
         yoGraphicsListRegistry.registerArtifact(listName, planePointArtifact);

         caption = namePrefix + "PlaneNormal";
         yoPlaneNormal = new YoFrameVector(caption, worldFrame, registry);
         YoGraphicVector planeNormalViz =
               new YoGraphicVector(caption, yoPlanePoint, yoPlaneNormal, YoAppearance.Blue());
         planeNormalViz.setVisible(VISUALIZE);
         yoGraphicsListRegistry.registerYoGraphic(listName, planeNormalViz);

         caption = namePrefix + "LineOfRotationGeometric";
         yoLineOfRotation = new YoFrameLineSegment2d(caption, "", "", worldFrame, registry);
         YoArtifactLineSegment2d lineOfRotationArtifact =
               new YoArtifactLineSegment2d(caption, yoLineOfRotation, Color.GREEN, 0.01, 0.01);
         lineOfRotationArtifact.setVisible(VISUALIZE);
         yoGraphicsListRegistry.registerArtifact(listName, lineOfRotationArtifact);
      }
      else
      {
         yoPlanePoint = null;
         yoPlaneNormal = null;
         yoLineOfRotation = null;
      }
   }

   private final Point3D planePoint = new Point3D();
   private final Vector3D normal = new Vector3D();

   private final Vector3D lineOfContactVector = new Vector3D();
   private final FrameVector3D lineOfContact = new FrameVector3D();
   private final FrameVector3D footNormal = new FrameVector3D();
   private final Vector3D footNormalVector = new Vector3D();

   private final FramePoint2D centerOfRotation2d = new FramePoint2D();
   private final FrameVector2D lineOfRotation2d = new FrameVector2D();

   @Override
   public void compute(FramePoint2D desiredCoP, FramePoint2D centerOfPressure)
   {
      centerOfPressure.checkReferenceFrameMatch(soleFrame);
      cop.setIncludingFrame(centerOfPressure, 0.0);
      cop.changeFrame(worldFrame);

      // assuming flat ground:
      // This can be updated to do some least square best fit for a plane through all
      // measured cops. For now assume that the surface normal is [0.0, 0.0, 1.0] in world
      copFiltered.update(cop);
      planePoint.set(copFiltered);
      normal.set(0.0, 0.0, 1.0);

      normal.normalize();
      groundPlanePoint.set(planePoint);
      groundPlaneNormal.setIncludingFrame(worldFrame, normal);

      // intersect the foot plane and the ground plane
      footNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0);
      footNormal.changeFrame(worldFrame);
      footNormalVector.set(footNormal);
      footNormalVector.normalize();
      lineOfContactVector.cross(normal, footNormalVector);
      if (lineOfContactVector.epsilonEquals(zero, 10E-12)) return;
      lineOfContact.setIncludingFrame(worldFrame, lineOfContactVector);

      // compute the angle between ground plane and foot plane
      double cosAlpha = normal.dot(footNormalVector);
      cosAlpha = Math.abs(cosAlpha);
      double alpha = Math.acos(cosAlpha);
      angleFootGround.set(alpha);
      footRotating.set(alpha > angleTreshold.getDoubleValue());

      centerOfRotation2d.set(cop);
      lineOfRotation2d.set(lineOfContact);
      lineOfRotationInWorldFrame.set(centerOfRotation2d, lineOfRotation2d);

      lineOfRotationInSoleFrame.setIncludingFrame(lineOfRotationInWorldFrame);
      lineOfRotationInSoleFrame.changeFrameAndProjectToXYPlane(soleFrame);
      lineOfRotationInSoleFrame.setPoint(centerOfPressure);

      if (yoLineOfRotation != null)
      {
         footPolygonInWorld.setIncludingFrameAndUpdate(defaultFootPolygon);
         footPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

         
         frameConvexPolygonWithLineIntersector2d.intersectWithLine(footPolygonInWorld, lineOfRotationInWorldFrame);
         if (frameConvexPolygonWithLineIntersector2d.getIntersectionResult() == IntersectionResult.NO_INTERSECTION
               || frameConvexPolygonWithLineIntersector2d.getIntersectionResult() == IntersectionResult.POINT_INTERSECTION
               || frameConvexPolygonWithLineIntersector2d.getIntersectionPointOne()
                                                         .epsilonEquals(frameConvexPolygonWithLineIntersector2d.getIntersectionPointTwo(), 1e-3))
         {
            yoLineOfRotation.setToNaN();
         }
         else
         {
            lineSegmentOfRotation.setIncludingFrame(frameConvexPolygonWithLineIntersector2d.getIntersectionPointOne(),
                                                    frameConvexPolygonWithLineIntersector2d.getIntersectionPointTwo());
            yoLineOfRotation.set(lineSegmentOfRotation);
         }
      }

      if (yoPlanePoint != null)
      {
         yoPlanePoint.set(groundPlanePoint);
      }

      if (yoPlaneNormal != null)
      {
         yoPlaneNormal.set(groundPlaneNormal);
      }
   }

   @Override
   public void reset()
   {
      copFiltered.reset();
      footRotating.set(false);

      if (yoLineOfRotation != null)
      {
         yoLineOfRotation.setToNaN();
      }
      if (yoPlanePoint != null)
      {
         yoPlanePoint.setToNaN();
      }
      if (yoPlaneNormal != null)
      {
         yoPlaneNormal.setToNaN();
      }
   }

   @Override
   public boolean isFootRotating()
   {
      return footRotating.getBooleanValue();
   }

   @Override
   public void getLineOfRotation(FrameLine2D lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }
}
