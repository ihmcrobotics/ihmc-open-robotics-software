package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;

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
   private final static boolean VISUALIZE = true;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private static final Vector3d zero = new Vector3d(0.0, 0.0, 0.0);
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final static double defaultCopAlpha = 0.99;
   private final static double defaultAngleThreshold = 5.0 * Math.PI/180.0;

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d defaultFootPolygon;

   private final FramePoint groundPlanePoint = new FramePoint();
   private final FrameVector groundPlaneNormal = new FrameVector();
   private final FrameLine lineOfRotation = new FrameLine(worldFrame);
   private final FrameLine2d lineOfRotationInSoleFrame = new FrameLine2d();
   private final FrameLine2d lineOfRotationInWorldFrame = new FrameLine2d();
   private final FrameConvexPolygon2d footPolygonInWorld = new FrameConvexPolygon2d();

   private final FramePoint cop = new FramePoint();
   private final DoubleYoVariable copAlpha;
   private final AlphaFilteredYoFramePoint copFiltered;

   private final FrameLineSegment2d lineSegmentOfRotation = new FrameLineSegment2d();
   private YoFrameLineSegment2d yoLineOfRotation = null;

   private YoFramePoint yoPlanePoint = null;
   private YoFrameVector yoPlaneNormal = null;

   private final DoubleYoVariable angleFootGround;
   private final DoubleYoVariable angleTreshold;

   private final BooleanYoVariable footRotating;

   public GeometricFootRotationCalculator(String namePrefix,
         ContactablePlaneBody contactableFoot,
         YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());

      angleFootGround = new DoubleYoVariable(namePrefix + "AngleFootGround", registry);
      angleTreshold = new DoubleYoVariable(namePrefix + "AngleTresholdFootRotation", registry);
      angleTreshold.set(defaultAngleThreshold);
      footRotating = new BooleanYoVariable(namePrefix + "Rotating", registry);

      copAlpha = new DoubleYoVariable(namePrefix + "copAlpha", registry);
      copAlpha.set(defaultCopAlpha);
      copFiltered = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint(namePrefix + "CoPFiltered",
            "", registry, copAlpha, worldFrame);

      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         String caption = "";

         caption = namePrefix + "PlanePoint";
         yoPlanePoint = new YoFramePoint(caption, worldFrame, registry);
         YoGraphicPosition planePointViz =
               new YoGraphicPosition(caption, yoPlanePoint, 0.005, YoAppearance.Blue(), GraphicType.SOLID_BALL);
         yoGraphicsListRegistry.registerArtifact(caption, planePointViz.createArtifact());

         caption = namePrefix + "PlaneNormal";
         yoPlaneNormal = new YoFrameVector(caption, worldFrame, registry);
         YoGraphicVector planeNormalViz =
               new YoGraphicVector(caption, yoPlanePoint, yoPlaneNormal, 0.5, YoAppearance.Blue(), true, 0.05);
         yoGraphicsListRegistry.registerYoGraphic(caption, planeNormalViz);

         caption = namePrefix + "LineOfRotationGeometric";
         yoLineOfRotation = new YoFrameLineSegment2d(caption, "", "", worldFrame, registry);
         YoArtifactLineSegment2d lineOfRotationArtifact =
               new YoArtifactLineSegment2d(caption, yoLineOfRotation, Color.GREEN, 0.01, 0.01);
         yoGraphicsListRegistry.registerArtifact(caption, lineOfRotationArtifact);
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

      lineOfRotation.setOrigin(cop);
      lineOfRotation.setDirection(lineOfContact);

      centerOfRotation2d.setByProjectionOntoXYPlane(lineOfRotation.getFrameOrigin());
      lineOfRotation2d.setByProjectionOntoXYPlane(lineOfRotation.getFrameDirection());
      lineOfRotationInWorldFrame.set(centerOfRotation2d, lineOfRotation2d);

      lineOfRotationInSoleFrame.setIncludingFrame(lineOfRotationInWorldFrame);
      lineOfRotationInSoleFrame.changeFrameAndProjectToXYPlane(soleFrame);

      if (yoLineOfRotation != null)
      {
         footPolygonInWorld.setIncludingFrameAndUpdate(defaultFootPolygon);
         footPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

         FramePoint2d[] intersections = footPolygonInWorld.intersectionWith(lineOfRotationInWorldFrame);
         if (intersections == null || intersections.length == 1 || intersections[0].epsilonEquals(intersections[1], 1.0e-3))
         {
            yoLineOfRotation.setToNaN();
         }
         else
         {
            lineSegmentOfRotation.setIncludingFrame(intersections);
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

   public void reset()
   {
      copFiltered.reset();
      footRotating.set(false);
   }

   public boolean isFootRotating()
   {
      return footRotating.getBooleanValue();
   }

   public void getLineOfRotation(FrameLine2d lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }

}
