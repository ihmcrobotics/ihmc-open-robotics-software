package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * The FootRotationCalculator is a tool to detect if the foot is rotating around a steady line of
 * rotation. It is used in the PartialFootholdControlModule to determine if the current foothold is
 * only partial and if the foot support polygon should be shrunk. (In this class: LoR = Line of
 * Rotation & CoR = Center of Rotation)
 * 
 * @author Sylvain
 */
public class VelocityFootRotationCalculator implements FootRotationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /** Alpha filter to filter the foot angular velocity. */
   private final YoDouble angularVelocityFilterBreakFrequency;
   private final YoDouble angularVelocityAlphaFilter;
   private final double controllerDt;
   /** Foot filtered angular velocity in the sole frame. The yaw rate is intentionally ignored. */
   private final AlphaFilteredYoFrameVector2d footAngularVelocityFiltered;
   /** Foot angular velocity around the estimated line of rotation. */
   private final YoDouble angularVelocityAroundLineOfRotation;
   /** Point located on the line of rotation. The measured center of pressure is used to compute it. */

   /** Alpha filter to filter the foot angular velocity. */
   private final YoDouble yoCenterOfRotationPositionAlphaFilter;
   private final AlphaFilteredYoFramePoint2d centerOfRotationFiltered;
   /** Alpha filter to filter the linear velocity of the center of rotation. */
   private final YoDouble yoCenterOfRotationVelocityAlphaFilter;
   /** Filtered data of the center of rotation linear velocity. */
   private final FilteredVelocityYoFrameVector2d centerOfRotationVelocityFiltered;
   /**
    * Linear velocity of the center of rotation that is transverse (perpendicular) to the line of
    * rotation.
    */
   private final YoDouble centerOfRotationTransverseVelocity;
   /**
    * Estimated line of rotation of the foot. It is actually here a line segment that remains contained
    * in the foot.
    */
   private final YoFrameLineSegment2D lineSegmentOfRotation;
   /** Absolute angle of the line of rotation. */
   private final YoDouble angleOfLineOfRotation;
   /** Alpha filter used to filter the yaw rate of the line of rotation. */
   private final YoDouble lineOfRotationAngularVelocityAlphaFilter;
   /** Filtered yaw rate of the line of rotation. */
   private final FilteredVelocityYoVariable lineOfRotationAngularVelocityFiltered;
   /** Amount that the foot drops or lifts around the axis of rotation */
   private final YoDouble footDropOrLift;

   /**
    * Threshold on the yaw rate of the line of rotation to determine whether or not the line of
    * rotation is stable.
    */
   private final YoDouble stableLoRAngularVelocityThreshold;
   private final YoBoolean isLineOfRotationStable;

   /**
    * Threshold on the transversal velocity of the CoR w.r.t. the LoR to determine whether or not the
    * CoR is stable.
    */
   private final YoDouble stableCoRLinearVelocityThreshold;
   private final YoBoolean isCenterOfRotationStable;

   /** Threshold on the foot angular velocity around the line of rotation. */
   private final YoDouble angularVelocityAroundLoRThreshold;
   private final YoBoolean isAngularVelocityAroundLoRPastThreshold;

   /** Threshold on the foot drop around the line of rotation. */
   private final YoDouble footDropThreshold;
   private final YoBoolean isFootDropPastThreshold;

   /** Main output of this class that informs on whether or not the foot is rotating. */
   private final YoBoolean isFootRotating;

   private final YoBoolean hasBeenInitialized;

   private final FrameVector2D angularVelocity = new FrameVector2D();

   private final FrameVector2D footAngularVelocityUnitVector = new FrameVector2D();

   private final FrameLine2D lineOfRotationInSoleFrame = new FrameLine2D();
   private final FrameLine2D lineOfRotationInWorldFrame = new FrameLine2D();

   private final FrameVector3D pointingBackwardVector = new FrameVector3D();

   private final ContactablePlaneBody rotatingBody;

   private final ReferenceFrame soleFrame;

   private final Twist bodyTwist = new Twist();
   private final FrameConvexPolygon2D footPolygonInSoleFrame = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D footPolygonInWorld = new FrameConvexPolygon2D();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   private final String namePrefix;

   public VelocityFootRotationCalculator(String namePrefix,
                                         double dt,
                                         ContactablePlaneBody rotatingFoot,
                                         ExplorationParameters explorationParameters,
                                         YoGraphicsListRegistry yoGraphicsListRegistry,
                                         YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.rotatingBody = rotatingFoot;
      this.soleFrame = rotatingFoot.getSoleFrame();
      this.controllerDt = dt;

      footPolygonInSoleFrame.setIncludingFrame(FrameVertex2DSupplier.asFrameVertex2DSupplier(rotatingFoot.getContactPoints2d()));

      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      angularVelocityAlphaFilter = new YoDouble(namePrefix + name + "AngularVelocityAlphaFilter", registry);
      angularVelocityFilterBreakFrequency = explorationParameters.getAngularVelocityFilterBreakFrequency();
      angularVelocityFilterBreakFrequency.addListener((v) ->
      {
         double freq = angularVelocityFilterBreakFrequency.getDoubleValue();
         angularVelocityAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(freq, controllerDt));
      });
      double freq = angularVelocityFilterBreakFrequency.getDoubleValue();
      angularVelocityAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(freq, controllerDt));
      footAngularVelocityFiltered = new AlphaFilteredYoFrameVector2d(namePrefix + "AngularVelocityFiltered",
                                                                     "",
                                                                     registry,
                                                                     angularVelocityAlphaFilter,
                                                                     soleFrame);

      yoCenterOfRotationPositionAlphaFilter = new YoDouble(namePrefix + "CoRPositionAlphaFilter", registry);
      centerOfRotationFiltered = new AlphaFilteredYoFramePoint2d(namePrefix + "CoRFiltered", "", registry, yoCenterOfRotationPositionAlphaFilter, soleFrame);
      yoCenterOfRotationVelocityAlphaFilter = new YoDouble(namePrefix + "CoRVelocityAlphaFilter", registry);
      centerOfRotationTransverseVelocity = new YoDouble(namePrefix + "CoRTransversalVelocity", registry);
      centerOfRotationVelocityFiltered = new FilteredVelocityYoFrameVector2d(namePrefix + "CoRVelocity",
                                                                             "",
                                                                             yoCenterOfRotationVelocityAlphaFilter,
                                                                             dt,
                                                                             registry,
                                                                             centerOfRotationFiltered);

      lineSegmentOfRotation = new YoFrameLineSegment2D(namePrefix + "LoRPosition", worldFrame, registry);
      angleOfLineOfRotation = new YoDouble(namePrefix + "AngleOfLoR", registry);
      lineOfRotationAngularVelocityAlphaFilter = new YoDouble(namePrefix + "LoRAngularVelocityAlphaFilter", registry);
      lineOfRotationAngularVelocityFiltered = new FilteredVelocityYoVariable(namePrefix + "LoRAngularVelocityFiltered",
                                                                             "",
                                                                             lineOfRotationAngularVelocityAlphaFilter,
                                                                             angleOfLineOfRotation,
                                                                             dt,
                                                                             registry);

      angularVelocityAroundLineOfRotation = new YoDouble(namePrefix + "AngularVelocityAroundLoR", registry);

      footDropOrLift = new YoDouble(namePrefix + "FootDropOrLift", registry);

      stableLoRAngularVelocityThreshold = explorationParameters.getStableLoRAngularVelocityThreshold();
      isLineOfRotationStable = new YoBoolean(namePrefix + "IsLoRStable", registry);

      stableCoRLinearVelocityThreshold = explorationParameters.getStableCoRLinearVelocityThreshold();
      isCenterOfRotationStable = new YoBoolean(namePrefix + "IsCoRStable", registry);

      angularVelocityAroundLoRThreshold = explorationParameters.getAngularVelocityAroundLoRThreshold();
      isAngularVelocityAroundLoRPastThreshold = new YoBoolean(namePrefix + "IsAngularVelocityAroundLoRPastThreshold", registry);

      footDropThreshold = explorationParameters.getFootDropThreshold();
      isFootDropPastThreshold = new YoBoolean(namePrefix + "IsFootDropPastThreshold", registry);

      isFootRotating = new YoBoolean(namePrefix + "RotatingVelocity", registry);

      hasBeenInitialized = new YoBoolean(namePrefix + "HasBeenInitialized", registry);

      angularVelocity.setToZero(soleFrame);
      lineOfRotationInSoleFrame.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0, 0.0);

      if (yoGraphicsListRegistry != null)
      {
         Artifact lineOfRotationArtifact = new YoArtifactLineSegment2d(namePrefix + "LineOfRotation", lineSegmentOfRotation, Color.ORANGE, 0.005, 0.01);
         lineOfRotationArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineOfRotationArtifact);
      }
   }

   @SuppressWarnings("unused")
   @Override
   public void compute(FramePoint2DReadOnly desiredCoP, FramePoint2DReadOnly centerOfPressure)
   {
      rotatingBody.getRigidBody().getBodyFixedFrame().getTwistOfFrame(bodyTwist);
      angularVelocity.setIncludingFrame(bodyTwist.getAngularPart());
      angularVelocity.changeFrameAndProjectToXYPlane(soleFrame);

      footAngularVelocityFiltered.update(angularVelocity);

      angleOfLineOfRotation.set(Math.atan2(footAngularVelocityFiltered.getY(), footAngularVelocityFiltered.getX()));
      lineOfRotationAngularVelocityFiltered.updateForAngles();

      //      copError2d.setToZero(soleFrame);
      //      copError2d.sub(desiredCoP, centerOfPressure);
      //
      //      yoCoPErrorFiltered.update(copError2d);
      //      yoCoPErrorPerpendicularToRotation.set(yoCoPErrorFiltered.cross(footAngularVelocityUnitVector));

      footAngularVelocityUnitVector.setIncludingFrame(footAngularVelocityFiltered);
      footAngularVelocityUnitVector.normalize();

      centerOfRotationFiltered.update(centerOfPressure);
      centerOfRotationVelocityFiltered.update();
      centerOfRotationTransverseVelocity.set(centerOfRotationVelocityFiltered.cross(footAngularVelocityUnitVector));

      if (!hasBeenInitialized.getBooleanValue())
      {
         hasBeenInitialized.set(true);
         return;
      }

      // Compute Foot Drop or Lift...
      pointingBackwardVector.setIncludingFrame(soleFrame, footAngularVelocityUnitVector.getY(), -footAngularVelocityUnitVector.getX(), 0.0);
      pointingBackwardVector.normalize();
      pointingBackwardVector.scale(0.15); // FIXME magic number?
      pointingBackwardVector.changeFrame(worldFrame);

      footDropOrLift.set(pointingBackwardVector.getZ());
      angularVelocityAroundLineOfRotation.set(footAngularVelocityFiltered.length());

      updateFootRotationEstimate();

      if (isFootRotating.getBooleanValue())
      {
         lineOfRotationInSoleFrame.set(centerOfRotationFiltered, footAngularVelocityFiltered);

         lineOfRotationInWorldFrame.setIncludingFrame(lineOfRotationInSoleFrame);
         lineOfRotationInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);

         intersectLineOfRotationWithFootPolygon();
      }
      else
      {
         lineSegmentOfRotation.setToNaN();
      }
   }

   private void updateFootRotationEstimate()
   {
      isFootDropPastThreshold.set(footDropOrLift.getDoubleValue() < footDropThreshold.getDoubleValue());

      isLineOfRotationStable.set(Math.abs(lineOfRotationAngularVelocityFiltered.getDoubleValue()) < stableLoRAngularVelocityThreshold.getDoubleValue());

      isCenterOfRotationStable.set(Math.abs(centerOfRotationTransverseVelocity.getDoubleValue()) < stableCoRLinearVelocityThreshold.getDoubleValue());

      isAngularVelocityAroundLoRPastThreshold.set(angularVelocityAroundLineOfRotation.getDoubleValue() > angularVelocityAroundLoRThreshold.getDoubleValue());

      isFootRotating.set(isLineOfRotationStable.getBooleanValue() && isCenterOfRotationStable.getBooleanValue()
                         && isAngularVelocityAroundLoRPastThreshold.getBooleanValue() && isFootDropPastThreshold.getBooleanValue());
   }

   private void intersectLineOfRotationWithFootPolygon()
   {
      footPolygonInWorld.setIncludingFrame(footPolygonInSoleFrame);
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
   public boolean isFootRotating()
   {
      return isFootRotating.getBooleanValue();
   }

   @Override
   public void getLineOfRotation(FrameLine2DBasics lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
      lineOfRotationToPack.changeFrameAndProjectToXYPlane(soleFrame);
   }

   @Override
   public void reset()
   {
      lineSegmentOfRotation.setToNaN();
      footAngularVelocityFiltered.reset();
      footAngularVelocityFiltered.setToNaN();
      centerOfRotationFiltered.reset();
      centerOfRotationFiltered.setToNaN();

      centerOfRotationVelocityFiltered.reset();
      centerOfRotationVelocityFiltered.setToNaN();

      angleOfLineOfRotation.set(0.0);
      lineOfRotationAngularVelocityFiltered.set(Double.NaN);
      lineOfRotationAngularVelocityFiltered.reset();

      angularVelocityAroundLineOfRotation.set(Double.NaN);
      isLineOfRotationStable.set(false);
      isCenterOfRotationStable.set(false);
      isFootRotating.set(false);

      hasBeenInitialized.set(false);
   }

   public void setAlphaFilter(double alpha)
   {
      angularVelocityAlphaFilter.set(alpha);
   }

   public void setFootAngularVelocityThreshold(double threshold)
   {
      angularVelocityAroundLoRThreshold.set(threshold);
   }

   public void setStableAngularVelocityThreshold(double threshold)
   {
      stableLoRAngularVelocityThreshold.set(threshold);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(namePrefix + "LineOfRotation",
                                                                                    lineSegmentOfRotation,
                                                                                    ColorDefinitions.Orange()));
      return group;
   }
}
