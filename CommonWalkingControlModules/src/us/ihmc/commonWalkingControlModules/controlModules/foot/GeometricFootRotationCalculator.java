package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d.IntersectionResult;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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

   private static final Vector3d zero = new Vector3d(0.0, 0.0, 0.0);
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d defaultFootPolygon;

   private final FramePoint groundPlanePoint = new FramePoint();
   private final FrameVector groundPlaneNormal = new FrameVector();
   private final FrameLine2d lineOfRotationInSoleFrame = new FrameLine2d();
   private final FrameLine2d lineOfRotationInWorldFrame = new FrameLine2d();
   private final FrameConvexPolygon2d footPolygonInWorld = new FrameConvexPolygon2d();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   private final FramePoint cop = new FramePoint();
   private final DoubleYoVariable copAlpha;
   private final AlphaFilteredYoFramePoint copFiltered;
   private final FrameLineSegment2d lineSegmentOfRotation = new FrameLineSegment2d();

   private final YoFrameLineSegment2d yoLineOfRotation;
   private final YoFramePoint yoPlanePoint;
   private final YoFrameVector yoPlaneNormal;

   private final DoubleYoVariable angleFootGround;
   private final DoubleYoVariable angleTreshold;
   private final BooleanYoVariable footRotating;

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

      angleFootGround = new DoubleYoVariable(namePrefix + "AngleToGround", registry);
      angleTreshold = explorationParameters.getGeometricDetectionAngleThreshold();
      footRotating = new BooleanYoVariable(namePrefix + "RotatingGeometry", registry);

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

   private final Point3d planePoint = new Point3d();
   private final Vector3d normal = new Vector3d();

   private final Vector3d lineOfContactVector = new Vector3d();
   private final FrameVector lineOfContact = new FrameVector();
   private final FrameVector footNormal = new FrameVector();
   private final Vector3d footNormalVector = new Vector3d();

   private final FramePoint2d centerOfRotation2d = new FramePoint2d();
   private final FrameVector2d lineOfRotation2d = new FrameVector2d();

   @Override
   public void compute(FramePoint2d desiredCoP, FramePoint2d centerOfPressure)
   {
      centerOfPressure.checkReferenceFrameMatch(soleFrame);
      cop.setXYIncludingFrame(centerOfPressure);
      cop.changeFrame(worldFrame);

      // assuming flat ground:
      // This can be updated to do some least square best fit for a plane through all
      // measured cops. For now assume that the surface normal is [0.0, 0.0, 1.0] in world
      copFiltered.update(cop);
      copFiltered.getPoint(planePoint);
      normal.set(0.0, 0.0, 1.0);

      normal.normalize();
      groundPlanePoint.set(planePoint);
      groundPlaneNormal.setIncludingFrame(worldFrame, normal);

      // intersect the foot plane and the ground plane
      footNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0);
      footNormal.changeFrame(worldFrame);
      footNormal.getVector(footNormalVector);
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

      centerOfRotation2d.setByProjectionOntoXYPlane(cop);
      lineOfRotation2d.setByProjectionOntoXYPlane(lineOfContact);
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
            yoLineOfRotation.setFrameLineSegment2d(lineSegmentOfRotation);
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
   public void getLineOfRotation(FrameLine2d lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }
}
