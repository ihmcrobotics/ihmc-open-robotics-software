package us.ihmc.commonWalkingControlModules.controlModules.foot;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class can detect if the foot is on partial support. The idea is to keep track of where the
 * CoP has been and use those points to compute the ground plane. If a point on the foot drops below
 * that plane it can be assumed that there is no ground under that point.
 *
 * @author Georg
 */
public class GeometricFootRotationCalculator implements FootRotationCalculator
{
   private final static boolean VISUALIZE = false;

   private static final Vector3D zero = new Vector3D(0.0, 0.0, 0.0);
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame soleFrame;
   private final FrameConvexPolygon2D defaultFootPolygon;

   private final FrameLine2D lineOfRotationInSoleFrame = new FrameLine2D();
   private final FrameLine2D lineOfRotationInWorldFrame = new FrameLine2D();
   private final FrameConvexPolygon2D footPolygonInWorld = new FrameConvexPolygon2D();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   private final FramePoint3D measuredCoPInWorld = new FramePoint3D();
   private final AlphaFilteredYoFramePoint measuredCoPFiltered;

   private final YoFrameLineSegment2D lineSegmentOfRotation;
   private final YoFrameVector3D groundPlaneNormal;

   private final YoDouble angleFootGround;
   private final YoDouble angleThreshold;
   private final YoBoolean footRotating;

   private final String namePrefix;

   public GeometricFootRotationCalculator(String namePrefix,
                                          ContactablePlaneBody contactableFoot,
                                          ExplorationParameters explorationParameters,
                                          YoGraphicsListRegistry yoGraphicsListRegistry,
                                          YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));

      angleFootGround = new YoDouble(namePrefix + "AngleToGround", registry);
      angleThreshold = explorationParameters.getGeometricDetectionAngleThreshold();
      footRotating = new YoBoolean(namePrefix + "RotatingGeometry", registry);

      YoDouble copAlpha = explorationParameters.getGeometricDetectionPlanePointAlpha();
      measuredCoPFiltered = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint(namePrefix + "CoPFiltered", "", registry, copAlpha, worldFrame);

      groundPlaneNormal = new YoFrameVector3D(namePrefix + "PlaneNormal", worldFrame, registry);
      lineSegmentOfRotation = new YoFrameLineSegment2D(namePrefix + "LineOfRotationGeometric", "", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         String listName = getClass().getSimpleName();
         ArtifactList artifactList = new ArtifactList(listName);
         YoGraphicsList graphicsList = new YoGraphicsList(listName);

         YoGraphicPosition planePointViz = new YoGraphicPosition(namePrefix + "PlanePoint",
                                                                 measuredCoPFiltered,
                                                                 0.005,
                                                                 YoAppearance.Blue(),
                                                                 GraphicType.SOLID_BALL);

         YoGraphicVector planeNormalViz = new YoGraphicVector(namePrefix + "PlaneNormal", measuredCoPFiltered, groundPlaneNormal, YoAppearance.Blue());

         YoArtifactLineSegment2d lineOfRotationArtifact = new YoArtifactLineSegment2d(namePrefix + "LineOfRotationGeometric",
                                                                                      lineSegmentOfRotation,
                                                                                      Color.GREEN,
                                                                                      0.01,
                                                                                      0.01);

         graphicsList.add(planeNormalViz);
         artifactList.add(planePointViz.createArtifact());
         artifactList.add(lineOfRotationArtifact);

         graphicsList.setVisible(VISUALIZE);
         artifactList.setVisible(VISUALIZE);

         yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   private final FrameVector3D lineOfContact = new FrameVector3D();
   private final FrameVector3D footNormal = new FrameVector3D();

   @Override
   public void compute(FramePoint2DReadOnly desiredCoP, FramePoint2DReadOnly measuredCoP)
   {
      measuredCoPInWorld.setMatchingFrame(measuredCoP, 0.0);

      // assuming flat ground:
      // This can be updated to do some least square best fit for a plane through all
      // measured cops. For now assume that the surface normal is [0.0, 0.0, 1.0] in world
      measuredCoPFiltered.update(measuredCoPInWorld);
      groundPlaneNormal.set(worldFrame, 0.0, 0.0, 1.0);

      // intersect the foot plane and the ground plane
      footNormal.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0);
      footNormal.normalize();
      footNormal.changeFrame(worldFrame);
      lineOfContact.cross(groundPlaneNormal, footNormal);
      if (lineOfContact.epsilonEquals(zero, 1E-11))
         return;

      // compute the angle between ground plane and foot plane
      double cosAlpha = Math.abs(groundPlaneNormal.dot(footNormal));
      double alpha = Math.acos(cosAlpha);
      angleFootGround.set(alpha);
      footRotating.set(alpha > angleThreshold.getDoubleValue());

      lineOfRotationInWorldFrame.set(measuredCoPInWorld, lineOfContact);

      lineOfRotationInSoleFrame.setIncludingFrame(lineOfRotationInWorldFrame);
      lineOfRotationInSoleFrame.changeFrameAndProjectToXYPlane(soleFrame);

      intersectLineOfRotationWithFootPolygon();
   }

   private void intersectLineOfRotationWithFootPolygon()
   {
      footPolygonInWorld.setIncludingFrame(defaultFootPolygon);
      footPolygonInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      frameConvexPolygonWithLineIntersector2d.intersectWithLine(footPolygonInWorld, lineOfRotationInWorldFrame);

      if (FootRotationCalculator.isIntersectionValid(frameConvexPolygonWithLineIntersector2d))
      {
         lineSegmentOfRotation.set(frameConvexPolygonWithLineIntersector2d.getIntersectionPointOne(),
                                   frameConvexPolygonWithLineIntersector2d.getIntersectionPointTwo());
      }
      else
      {
         lineSegmentOfRotation.setToNaN();
      }
   }

   @Override
   public void reset()
   {
      measuredCoPFiltered.reset();
      footRotating.set(false);

      lineSegmentOfRotation.setToNaN();
      groundPlaneNormal.setToNaN();
   }

   @Override
   public boolean isFootRotating()
   {
      return footRotating.getBooleanValue();
   }

   @Override
   public void getLineOfRotation(FrameLine2DBasics lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicArrow3D(namePrefix + "PlaneNormal", measuredCoPFiltered, groundPlaneNormal, 1.0, ColorDefinitions.Blue()));
      group.addChild(newYoGraphicPoint2D(namePrefix + "PlanePoint", measuredCoPFiltered, 0.01, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_FILLED));
      group.addChild(newYoGraphicLineSegment2DDefinition(namePrefix + "LineOfRotationGeometric", lineSegmentOfRotation, ColorDefinitions.Green()));
      group.setVisible(VISUALIZE);
      return group;
   }
}
