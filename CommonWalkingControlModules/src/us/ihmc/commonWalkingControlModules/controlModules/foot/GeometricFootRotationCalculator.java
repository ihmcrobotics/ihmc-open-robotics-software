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
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
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

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2d defaultFootPolygon;

   private final FramePoint groundPlanePoint = new FramePoint();
   private final FrameVector groundPlaneNormal = new FrameVector();
   private final FrameLine lineOfRotation = new FrameLine(worldFrame);
   private final FrameLine2d lineOfRotationInSoleFrame = new FrameLine2d();
   private final FrameLine2d lineOfRotationInWorldFrame = new FrameLine2d();
   private final FrameConvexPolygon2d copEnclosingPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d footPolygonInWorld = new FrameConvexPolygon2d();

   private final FrameLineSegment2d lineSegmentOfRotation = new FrameLineSegment2d();
   private YoFrameLineSegment2d yoLineOfRotation = null;

//   private final FramePoint2d cop2d = new FramePoint2d();
   private final FramePoint cop = new FramePoint();
   private double numberOfMeasurements = 0.0;

   private YoFrameConvexPolygon2d yoCopEnclosingPolygon = null;
   private YoFramePoint yoPlanePoint = null;
   private YoFrameVector yoPlaneNormal = null;

   private final DoubleYoVariable[] footVertexDrops;
   private final DoubleYoVariable totalSquaredDrop;
   private final DoubleYoVariable dropTreshold;
   private final static double defaultDropThreshold = 0.015;
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

      int numberOfVertices = defaultFootPolygon.getNumberOfVertices();
      footVertexDrops = new DoubleYoVariable[numberOfVertices];
      for (int i = 0; i < numberOfVertices; i++)
      {
         String name = namePrefix + "CornerDrop_" + i;
         footVertexDrops[i] = new DoubleYoVariable(name, registry);
      }

      dropTreshold = new DoubleYoVariable(namePrefix + "CornerDropThreshold", registry);
      dropTreshold.set(defaultDropThreshold);
      footRotating = new BooleanYoVariable(namePrefix + "Rotating", registry);
      totalSquaredDrop = new DoubleYoVariable(namePrefix + "TotalSquaredDrop", registry);

      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         String caption = namePrefix + "CopEnclosingPolygon";
//         yoCopEnclosingPolygon = new YoFrameConvexPolygon2d(caption, "", worldFrame, 100, registry);
//         YoArtifactPolygon dynamicGraphicYoPolygonArtifact =
//               new YoArtifactPolygon(caption, yoCopEnclosingPolygon, Color.GREEN, false);
//         yoGraphicsListRegistry.registerArtifact(caption, dynamicGraphicYoPolygonArtifact);

         caption = namePrefix + "PlanePoint";
         yoPlanePoint = new YoFramePoint(caption, worldFrame, registry);
         YoGraphicPosition planePointViz =
               new YoGraphicPosition(caption, yoPlanePoint, 0.01, YoAppearance.Blue(), GraphicType.SOLID_BALL);
         yoGraphicsListRegistry.registerArtifact(caption, planePointViz.createArtifact());

         caption = namePrefix + "PlaneNormal";
         yoPlaneNormal = new YoFrameVector(caption, worldFrame, registry);
         YoGraphicVector planeNormalViz =
               new YoGraphicVector(caption, yoPlanePoint, yoPlaneNormal, 1.0, YoAppearance.Blue(), true, 0.01);
         yoGraphicsListRegistry.registerYoGraphic(caption, planeNormalViz);

         caption = namePrefix + "LineOfRotationGeometric";
         yoLineOfRotation = new YoFrameLineSegment2d(caption, "", "", worldFrame, registry);
         YoArtifactLineSegment2d lineOfRotationArtifact =
               new YoArtifactLineSegment2d(caption, yoLineOfRotation, Color.GREEN, 0.005, 0.01);
         yoGraphicsListRegistry.registerArtifact(caption, lineOfRotationArtifact);
      }
   }

   private final Point3d copPosition = new Point3d();
   private final Point3d planePosition = new Point3d();
   private final Vector3d normal = new Vector3d();

   private final FramePoint2d footVertex2d = new FramePoint2d();
   private final FramePoint footVertex = new FramePoint();
   private final Point3d footVertexPosition = new Point3d();
   private final Vector3d footVertexVector = new Vector3d();

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
      cop.getPoint(copPosition);
      groundPlanePoint.getPoint(planePosition);
      copPosition.scale(1.0 / (numberOfMeasurements + 1.0));
      planePosition.scale(numberOfMeasurements / (numberOfMeasurements + 1.0));
      planePosition.add(copPosition);
      numberOfMeasurements++;
      normal.set(0.0, 0.0, 1.0);

      groundPlanePoint.set(planePosition);
      groundPlaneNormal.setIncludingFrame(worldFrame, normal);

      // now go through all the foot corners and see if they are below the ground surface
      boolean cornerBelowGround = false;
      double totalSquaredDrop = 0.0;
      for (int i = 0; i < defaultFootPolygon.getNumberOfVertices(); i++)
      {
         defaultFootPolygon.getFrameVertex(i, footVertex2d);
         footVertex.setXYIncludingFrame(footVertex2d);
         footVertex.changeFrame(worldFrame);

         footVertex.getPoint(footVertexPosition);
         footVertexVector.set(footVertexPosition);
         footVertexVector.sub(planePosition);
         double distance = normal.dot(footVertexVector) / normal.lengthSquared();
         footVertexDrops[i].set(-distance);

         if (distance < -dropTreshold.getDoubleValue())
         {
            cornerBelowGround = true;
         }

         distance = Math.abs(distance);
         totalSquaredDrop += distance * distance;
      }

      totalSquaredDrop = Math.sqrt(totalSquaredDrop);
      this.totalSquaredDrop.set(totalSquaredDrop);

      if (cornerBelowGround)
      {
         footRotating.set(true);
      }
      else
      {
         footRotating.set(false);
      }

      // intersect the foot plane and the ground plane
      footNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0);
      footNormal.changeFrame(worldFrame);
      footNormal.getVector(footNormalVector);
      lineOfContactVector.cross(normal, footNormalVector);
      lineOfContact.setIncludingFrame(worldFrame, lineOfContactVector);

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


      // instead of computing point of intersection use the cop

//      cop2d.setIncludingFrame(centerOfPressure);
//      cop2d.changeFrameAndProjectToXYPlane(worldFrame);
//      if (!copEnclosingPolygon.isPointInside(cop2d))
//      {
//         copEnclosingPolygon.addVertex(cop2d);
//         copEnclosingPolygon.update();
//      }
//
//      copEnclosingPolygon.changeFrameAndProjectToXYPlane(worldFrame);
//      if (yoCopEnclosingPolygon != null)
//      {
//         yoCopEnclosingPolygon.setFrameConvexPolygon2d(copEnclosingPolygon);
//      }

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
      copEnclosingPolygon.clear();
      copEnclosingPolygon.update();
      if (yoCopEnclosingPolygon != null)
      {
         yoCopEnclosingPolygon.hide();
      }
      numberOfMeasurements = 0.0;
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
