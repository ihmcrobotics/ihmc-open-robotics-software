package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d.IntersectionResult;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * The FootRotationCalculator is a tool to detect if the foot is rotating around a steady line of rotation.
 * It is used in the PartialFootholdControlModule to determine if the current foothold is only partial and if the foot support polygon should be shrunk.
 * (In this class: LoR = Line of Rotation & CoR = Center of Rotation)
 * @author Sylvain
 *
 */
public class VelocityFootRotationCalculator implements FootRotationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE = true;

   private final String name = getClass().getSimpleName();

   private final String generalDescription = "LoR = Line of Rotation & CoR = Center of Rotation";

   private final YoVariableRegistry registry;

   /** Alpha filter to filter the foot angular velocity. */
   private final YoDouble anglularVelocityFilterBreakFrequeny;
   private final YoDouble yoAngularVelocityAlphaFilter;
   private final double controllerDt;
   /** Foot filtered angular velocity in the sole frame. The yaw rate is intentionally ignored. */
   private final AlphaFilteredYoFrameVector2d yoFootAngularVelocityFiltered;
   /** Foot angular velocity around the estimated line of rotation. */
   private final YoDouble yoAngularVelocityAroundLoR;
   /** Point located on the line of rotation. The measured center of pressure is used to compute it. */

   /** Alpha filter to filter the foot angular velocity. */
   private final YoDouble yoCoRPositionAlphaFilter;
   private final AlphaFilteredYoFramePoint2d yoCoRPositionFiltered;
   /** Alpha filter to filter the linear velocity of the center of rotation. */
   private final YoDouble yoCoRVelocityAlphaFilter;
   /** Filtered data of the center of rotation linear velocity. */
   private final FilteredVelocityYoFrameVector2d yoCoRVelocityFiltered;
   /** Linear velocity of the center of rotation that is transversal (perpendicular) to the line of rotation. */
   private final YoDouble yoCoRTransversalVelocity;
   /** Estimated line of rotation of the foot. It is actually here a line segment that remains contained in the foot. */
   private final YoFrameLineSegment2d yoLineOfRotation;
   /** Absolute angle of the line of rotation. */
   private final YoDouble yoAngleOfLoR;
   /** Alpha filter used to filter the yaw rate of the line of rotation. */
   private final YoDouble yoLoRAngularVelocityAlphaFilter;
   /** Filtered yaw rate of the line of rotation. */
   private final FilteredVelocityYoVariable yoLoRAngularVelocityFiltered;
   /** Amount that the foot drops or lifts around the axis of rotation */
   private final YoDouble yoFootDropOrLift;

   /** Threshold on the yaw rate of the line of rotation to determine whether or not the line of rotation is stable. */
   private final YoDouble yoStableLoRAngularVelocityThreshold;
   private final YoBoolean yoIsLoRStable;

   /** Threshold on the transversal velocity of the CoR w.r.t. the LoR to determine whether or not the CoR is stable. */
   private final YoDouble yoStableCoRLinearVelocityThreshold;
   private final YoBoolean yoIsCoRStable;

   /** Threshold on the foot angular velocity around the line of rotation. */
   private final YoDouble yoAngularVelocityAroundLoRThreshold;
   private final YoBoolean yoIsAngularVelocityAroundLoRPastThreshold;

   /** Threshold on the foot drop around the line of rotation. */
   private final YoDouble yoFootDropThreshold;
   private final YoBoolean yoIsFootDropPastThreshold;

   /** Main output of this class that informs on wether or not the foot is rotating. */
   private final YoBoolean yoIsFootRotating;

   private final YoBoolean hasBeenInitialized;

   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector2D angularVelocity2d = new FrameVector2D();

   private final FrameVector2D footAngularVelocityUnitVector = new FrameVector2D();

   private final FramePoint2D centerOfRotation = new FramePoint2D();
   private final FrameLine2D lineOfRotationInSoleFrame = new FrameLine2D();
   private final FrameLine2D lineOfRotationInWorldFrame = new FrameLine2D();
   private final FrameLineSegment2D lineSegmentOfRotation = new FrameLineSegment2D();

   private final FrameVector3D pointingBackwardVector = new FrameVector3D();

   private final ContactablePlaneBody rotatingBody;

   private final ReferenceFrame soleFrame;

   private final Twist bodyTwist = new Twist();
   private final FrameConvexPolygon2d footPolygonInSoleFrame = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d footPolygonInWorldFrame = new FrameConvexPolygon2d();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public VelocityFootRotationCalculator(String namePrefix, double dt, ContactablePlaneBody rotatingFoot,
         ExplorationParameters explorationParameters, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.rotatingBody = rotatingFoot;
      this.soleFrame = rotatingFoot.getSoleFrame();
      this.controllerDt = dt;

      footPolygonInSoleFrame.setIncludingFrameAndUpdate(rotatingFoot.getContactPoints2d());

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      yoAngularVelocityAlphaFilter = new YoDouble(namePrefix + name + "AngularVelocityAlphaFilter", generalDescription, registry);
      anglularVelocityFilterBreakFrequeny = explorationParameters.getAngularVelocityFilterBreakFrequency();
      anglularVelocityFilterBreakFrequeny.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double freq = anglularVelocityFilterBreakFrequeny.getDoubleValue();
            double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(freq, controllerDt);
            yoAngularVelocityAlphaFilter.set(alpha);
         }
      });
      double freq = anglularVelocityFilterBreakFrequeny.getDoubleValue();
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(freq, controllerDt);
      yoAngularVelocityAlphaFilter.set(alpha);
      yoFootAngularVelocityFiltered = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d(namePrefix + "AngularVelocityFiltered", "",
            generalDescription, registry, yoAngularVelocityAlphaFilter, soleFrame);

      yoCoRPositionAlphaFilter = new YoDouble(namePrefix + "CoRPositionAlphaFilter", registry);
      yoCoRPositionFiltered = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(namePrefix + "CoRFiltered", "", generalDescription, registry,
            yoCoRPositionAlphaFilter, soleFrame);
      yoCoRVelocityAlphaFilter = new YoDouble(namePrefix + "CoRVelocityAlphaFilter", generalDescription, registry);
      yoCoRTransversalVelocity = new YoDouble(namePrefix + "CoRTransversalVelocity", generalDescription, registry);
      yoCoRVelocityFiltered = FilteredVelocityYoFrameVector2d.createFilteredVelocityYoFrameVector2d(namePrefix + "CoRVelocity", "", generalDescription,
            yoCoRVelocityAlphaFilter, dt, registry, yoCoRPositionFiltered);

      yoLineOfRotation = new YoFrameLineSegment2d(namePrefix + "LoRPosition", "", generalDescription, worldFrame, registry);
      yoAngleOfLoR = new YoDouble(namePrefix + "AngleOfLoR", generalDescription, registry);
      yoLoRAngularVelocityAlphaFilter = new YoDouble(namePrefix + "LoRAngularVelocityAlphaFilter", generalDescription, registry);
      yoLoRAngularVelocityFiltered = new FilteredVelocityYoVariable(namePrefix + "LoRAngularVelocityFiltered", generalDescription,
            yoLoRAngularVelocityAlphaFilter, yoAngleOfLoR, dt, registry);

      yoAngularVelocityAroundLoR = new YoDouble(namePrefix + "AngularVelocityAroundLoR", generalDescription, registry);

      yoFootDropOrLift = new YoDouble(namePrefix + "FootDropOrLift", generalDescription, registry);

      yoStableLoRAngularVelocityThreshold = explorationParameters.getStableLoRAngularVelocityThreshold();
      yoIsLoRStable = new YoBoolean(namePrefix + "IsLoRStable", generalDescription, registry);

      yoStableCoRLinearVelocityThreshold = explorationParameters.getStableCoRLinearVelocityThreshold();
      yoIsCoRStable = new YoBoolean(namePrefix + "IsCoRStable", generalDescription, registry);

      yoAngularVelocityAroundLoRThreshold = explorationParameters.getAngularVelocityAroundLoRThreshold();
      yoIsAngularVelocityAroundLoRPastThreshold = new YoBoolean(namePrefix + "IsAngularVelocityAroundLoRPastThreshold", generalDescription, registry);

      yoFootDropThreshold = explorationParameters.getFootDropThreshold();
      yoIsFootDropPastThreshold = new YoBoolean(namePrefix + "IsFootDropPastThreshold", generalDescription, registry);

      yoIsFootRotating = new YoBoolean(namePrefix + "RotatingVelocity", generalDescription, registry);

      hasBeenInitialized = new YoBoolean(namePrefix + "HasBeenInitialized", registry);

      angularVelocity2d.setToZero(soleFrame);
      lineOfRotationInSoleFrame.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0, 0.0);

      if (yoGraphicsListRegistry != null)
      {
         Artifact lineOfRotationArtifact = new YoArtifactLineSegment2d(namePrefix + "LineOfRotation", yoLineOfRotation, Color.ORANGE, 0.005, 0.01);
         lineOfRotationArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineOfRotationArtifact);
      }
   }

   @SuppressWarnings("unused")
   @Override
   public void compute(FramePoint2D desiredCoP, FramePoint2D centerOfPressure)
   {
      footPolygonInWorldFrame.setIncludingFrameAndUpdate(footPolygonInSoleFrame);
      footPolygonInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);

      rotatingBody.getRigidBody().getBodyFixedFrame().getTwistOfFrame(bodyTwist);
      bodyTwist.getAngularPart(angularVelocity);

      angularVelocity.changeFrame(soleFrame);
      angularVelocity.setZ(0.0);
      angularVelocity2d.setIncludingFrame(soleFrame, angularVelocity.getX(), angularVelocity.getY());

      yoFootAngularVelocityFiltered.update(angularVelocity2d);
      angularVelocity2d.setIncludingFrame(yoFootAngularVelocityFiltered);

      yoAngleOfLoR.set(Math.atan2(angularVelocity2d.getY(), angularVelocity2d.getX()));
      yoLoRAngularVelocityFiltered.updateForAngles();

//      copError2d.setToZero(soleFrame);
//      copError2d.sub(desiredCoP, centerOfPressure);
//
//      yoCoPErrorFiltered.update(copError2d);
//      yoCoPErrorPerpendicularToRotation.set(yoCoPErrorFiltered.cross(footAngularVelocityUnitVector));

      footAngularVelocityUnitVector.setIncludingFrame(yoFootAngularVelocityFiltered);
      footAngularVelocityUnitVector.normalize();

      yoCoRPositionFiltered.update(centerOfPressure);
      centerOfRotation.setIncludingFrame(yoCoRPositionFiltered);
      yoCoRVelocityFiltered.update();
      yoCoRTransversalVelocity.set(yoCoRVelocityFiltered.cross(footAngularVelocityUnitVector));

      if (!hasBeenInitialized.getBooleanValue())
      {
         hasBeenInitialized.set(true);
         return;
      }


      // Compute Foot Drop or Lift...
      pointingBackwardVector.setIncludingFrame(soleFrame, footAngularVelocityUnitVector.getY(), -footAngularVelocityUnitVector.getX(), 0.0);
      pointingBackwardVector.normalize();
      pointingBackwardVector.scale(0.15);
      pointingBackwardVector.changeFrame(worldFrame);

      yoFootDropOrLift.set(pointingBackwardVector.getZ());
      yoIsFootDropPastThreshold.set(yoFootDropOrLift.getDoubleValue() < yoFootDropThreshold.getDoubleValue());

      yoIsLoRStable.set(Math.abs(yoLoRAngularVelocityFiltered.getDoubleValue()) < yoStableLoRAngularVelocityThreshold.getDoubleValue());

      yoIsCoRStable.set(Math.abs(yoCoRTransversalVelocity.getDoubleValue()) < yoStableCoRLinearVelocityThreshold.getDoubleValue());

      yoAngularVelocityAroundLoR.set(yoFootAngularVelocityFiltered.length());
      yoIsAngularVelocityAroundLoRPastThreshold.set(yoAngularVelocityAroundLoR.getDoubleValue() > yoAngularVelocityAroundLoRThreshold.getDoubleValue());

      yoIsFootRotating.set(yoIsLoRStable.getBooleanValue() && yoIsCoRStable.getBooleanValue() && yoIsAngularVelocityAroundLoRPastThreshold.getBooleanValue() && yoIsFootDropPastThreshold.getBooleanValue());

      if (VISUALIZE || yoIsFootRotating.getBooleanValue())
      {
         lineOfRotationInSoleFrame.set(centerOfRotation, angularVelocity2d);
         lineOfRotationInWorldFrame.setIncludingFrame(lineOfRotationInSoleFrame);
         lineOfRotationInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);

         frameConvexPolygonWithLineIntersector2d.intersectWithLine(footPolygonInWorldFrame, lineOfRotationInWorldFrame);
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
      else
      {
         yoLineOfRotation.setToNaN();
      }
   }

   @Override
   public boolean isFootRotating()
   {
      return yoIsFootRotating.getBooleanValue();
   }

   @Override
   public void getLineOfRotation(FrameLine2D lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }

   @Override
   public void reset()
   {
      yoLineOfRotation.setToNaN();
      yoFootAngularVelocityFiltered.reset();
      yoFootAngularVelocityFiltered.setToNaN();
      yoCoRPositionFiltered.reset();
      yoCoRPositionFiltered.setToNaN();

      yoCoRVelocityFiltered.reset();
      yoCoRVelocityFiltered.setToNaN();

      yoAngleOfLoR.set(0.0);
      yoLoRAngularVelocityFiltered.set(Double.NaN);
      yoLoRAngularVelocityFiltered.reset();

      yoAngularVelocityAroundLoR.set(Double.NaN);
      yoIsLoRStable.set(false);
      yoIsCoRStable.set(false);
      yoIsFootRotating.set(false);

      hasBeenInitialized.set(false);
   }

   public void setAlphaFilter(double alpha)
   {
      yoAngularVelocityAlphaFilter.set(alpha);
   }

   public void setFootAngularVelocityThreshold(double threshold)
   {
      yoAngularVelocityAroundLoRThreshold.set(threshold);
   }

   public void setStableAngularVelocityThreshold(double threshold)
   {
      yoStableLoRAngularVelocityThreshold.set(threshold);
   }
}
