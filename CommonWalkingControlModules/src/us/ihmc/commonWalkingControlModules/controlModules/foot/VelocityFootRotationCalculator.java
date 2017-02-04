package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d.IntersectionResult;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

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
   private final DoubleYoVariable anglularVelocityFilterBreakFrequeny;
   private final DoubleYoVariable yoAngularVelocityAlphaFilter;
   private final double controllerDt;
   /** Foot filtered angular velocity in the sole frame. The yaw rate is intentionally ignored. */
   private final AlphaFilteredYoFrameVector2d yoFootAngularVelocityFiltered;
   /** Foot angular velocity around the estimated line of rotation. */
   private final DoubleYoVariable yoAngularVelocityAroundLoR;
   /** Point located on the line of rotation. The measured center of pressure is used to compute it. */

   /** Alpha filter to filter the foot angular velocity. */
   private final DoubleYoVariable yoCoRPositionAlphaFilter;
   private final AlphaFilteredYoFramePoint2d yoCoRPositionFiltered;
   /** Alpha filter to filter the linear velocity of the center of rotation. */
   private final DoubleYoVariable yoCoRVelocityAlphaFilter;
   /** Filtered data of the center of rotation linear velocity. */
   private final FilteredVelocityYoFrameVector2d yoCoRVelocityFiltered;
   /** Linear velocity of the center of rotation that is transversal (perpendicular) to the line of rotation. */
   private final DoubleYoVariable yoCoRTransversalVelocity;
   /** Estimated line of rotation of the foot. It is actually here a line segment that remains contained in the foot. */
   private final YoFrameLineSegment2d yoLineOfRotation;
   /** Absolute angle of the line of rotation. */
   private final DoubleYoVariable yoAngleOfLoR;
   /** Alpha filter used to filter the yaw rate of the line of rotation. */
   private final DoubleYoVariable yoLoRAngularVelocityAlphaFilter;
   /** Filtered yaw rate of the line of rotation. */
   private final FilteredVelocityYoVariable yoLoRAngularVelocityFiltered;
   /** Amount that the foot drops or lifts around the axis of rotation */
   private final DoubleYoVariable yoFootDropOrLift;

   private final Footstep currentDesiredFootstep;

   /** Threshold on the yaw rate of the line of rotation to determine whether or not the line of rotation is stable. */
   private final DoubleYoVariable yoStableLoRAngularVelocityThreshold;
   private final BooleanYoVariable yoIsLoRStable;

   /** Threshold on the transversal velocity of the CoR w.r.t. the LoR to determine whether or not the CoR is stable. */
   private final DoubleYoVariable yoStableCoRLinearVelocityThreshold;
   private final BooleanYoVariable yoIsCoRStable;

   /** Threshold on the foot angular velocity around the line of rotation. */
   private final DoubleYoVariable yoAngularVelocityAroundLoRThreshold;
   private final BooleanYoVariable yoIsAngularVelocityAroundLoRPastThreshold;

   /** Threshold on the foot drop around the line of rotation. */
   private final DoubleYoVariable yoFootDropThreshold;
   private final BooleanYoVariable yoIsFootDropPastThreshold;

   /** Main output of this class that informs on wether or not the foot is rotating. */
   private final BooleanYoVariable yoIsFootRotating;

   private final BooleanYoVariable hasBeenInitialized;

   private final FrameVector angularVelocity = new FrameVector();
   private final FrameVector2d angularVelocity2d = new FrameVector2d();

   private final FrameVector2d footAngularVelocityUnitVector = new FrameVector2d();

   private final FramePoint2d centerOfRotation = new FramePoint2d();
   private final FrameLine2d lineOfRotationInSoleFrame = new FrameLine2d();
   private final FrameLine2d lineOfRotationInWorldFrame = new FrameLine2d();
   private final FrameLineSegment2d lineSegmentOfRotation = new FrameLineSegment2d();

   private final FrameVector pointingBackwardVector = new FrameVector();

   private final ContactablePlaneBody rotatingBody;
   private final TwistCalculator twistCalculator;

   private final ReferenceFrame soleFrame;

   private final Twist bodyTwist = new Twist();
   private final FrameConvexPolygon2d footPolygonInSoleFrame = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d footPolygonInWorldFrame = new FrameConvexPolygon2d();
   private final FrameConvexPolygonWithLineIntersector2d frameConvexPolygonWithLineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public VelocityFootRotationCalculator(String namePrefix, double dt, ContactablePlaneBody rotatingFoot, TwistCalculator twistCalculator,
         ExplorationParameters explorationParameters, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.twistCalculator = twistCalculator;
      this.rotatingBody = rotatingFoot;
      this.soleFrame = rotatingFoot.getSoleFrame();
      this.controllerDt = dt;

      currentDesiredFootstep = new Footstep(rotatingFoot.getRigidBody(), null, soleFrame);

      footPolygonInSoleFrame.setIncludingFrameAndUpdate(rotatingFoot.getContactPoints2d());

      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      yoAngularVelocityAlphaFilter = new DoubleYoVariable(namePrefix + name + "AngularVelocityAlphaFilter", generalDescription, registry);
      anglularVelocityFilterBreakFrequeny = explorationParameters.getAngularVelocityFilterBreakFrequency();
      anglularVelocityFilterBreakFrequeny.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
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

      yoCoRPositionAlphaFilter = new DoubleYoVariable(namePrefix + "CoRPositionAlphaFilter", registry);
      yoCoRPositionFiltered = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(namePrefix + "CoRFiltered", "", generalDescription, registry,
            yoCoRPositionAlphaFilter, soleFrame);
      yoCoRVelocityAlphaFilter = new DoubleYoVariable(namePrefix + "CoRVelocityAlphaFilter", generalDescription, registry);
      yoCoRTransversalVelocity = new DoubleYoVariable(namePrefix + "CoRTransversalVelocity", generalDescription, registry);
      yoCoRVelocityFiltered = FilteredVelocityYoFrameVector2d.createFilteredVelocityYoFrameVector2d(namePrefix + "CoRVelocity", "", generalDescription,
            yoCoRVelocityAlphaFilter, dt, registry, yoCoRPositionFiltered);

      yoLineOfRotation = new YoFrameLineSegment2d(namePrefix + "LoRPosition", "", generalDescription, worldFrame, registry);
      yoAngleOfLoR = new DoubleYoVariable(namePrefix + "AngleOfLoR", generalDescription, registry);
      yoLoRAngularVelocityAlphaFilter = new DoubleYoVariable(namePrefix + "LoRAngularVelocityAlphaFilter", generalDescription, registry);
      yoLoRAngularVelocityFiltered = new FilteredVelocityYoVariable(namePrefix + "LoRAngularVelocityFiltered", generalDescription,
            yoLoRAngularVelocityAlphaFilter, yoAngleOfLoR, dt, registry);

      yoAngularVelocityAroundLoR = new DoubleYoVariable(namePrefix + "AngularVelocityAroundLoR", generalDescription, registry);

      yoFootDropOrLift = new DoubleYoVariable(namePrefix + "FootDropOrLift", generalDescription, registry);

      yoStableLoRAngularVelocityThreshold = explorationParameters.getStableLoRAngularVelocityThreshold();
      yoIsLoRStable = new BooleanYoVariable(namePrefix + "IsLoRStable", generalDescription, registry);

      yoStableCoRLinearVelocityThreshold = explorationParameters.getStableCoRLinearVelocityThreshold();
      yoIsCoRStable = new BooleanYoVariable(namePrefix + "IsCoRStable", generalDescription, registry);

      yoAngularVelocityAroundLoRThreshold = explorationParameters.getAngularVelocityAroundLoRThreshold();
      yoIsAngularVelocityAroundLoRPastThreshold = new BooleanYoVariable(namePrefix + "IsAngularVelocityAroundLoRPastThreshold", generalDescription, registry);

      yoFootDropThreshold = explorationParameters.getFootDropThreshold();
      yoIsFootDropPastThreshold = new BooleanYoVariable(namePrefix + "IsFootDropPastThreshold", generalDescription, registry);

      yoIsFootRotating = new BooleanYoVariable(namePrefix + "RotatingVelocity", generalDescription, registry);

      hasBeenInitialized = new BooleanYoVariable(namePrefix + "HasBeenInitialized", registry);

      angularVelocity2d.setToZero(soleFrame);
      lineOfRotationInSoleFrame.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0, 0.0);

      if (yoGraphicsListRegistry != null)
      {
         Artifact lineOfRotationArtifact = new YoArtifactLineSegment2d(namePrefix + "LineOfRotation", yoLineOfRotation, Color.ORANGE, 0.005, 0.01);
         lineOfRotationArtifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), lineOfRotationArtifact);
      }
   }

   public void setCurrentDesiredFootstep(Footstep currentDesiredFootstep)
   {
      this.currentDesiredFootstep.setPose(currentDesiredFootstep);
   }

   @SuppressWarnings("unused")
   @Override
   public void compute(FramePoint2d desiredCoP, FramePoint2d centerOfPressure)
   {
      footPolygonInWorldFrame.setIncludingFrameAndUpdate(footPolygonInSoleFrame);
      footPolygonInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);

      twistCalculator.getTwistOfBody(bodyTwist, rotatingBody.getRigidBody());
      bodyTwist.getAngularPart(angularVelocity);

      angularVelocity.changeFrame(soleFrame);
      angularVelocity.setZ(0.0);
      angularVelocity2d.setIncludingFrame(soleFrame, angularVelocity.getX(), angularVelocity.getY());

      yoFootAngularVelocityFiltered.update(angularVelocity2d);
      yoFootAngularVelocityFiltered.getFrameTuple2dIncludingFrame(angularVelocity2d);

      yoAngleOfLoR.set(Math.atan2(angularVelocity2d.getY(), angularVelocity2d.getX()));
      yoLoRAngularVelocityFiltered.updateForAngles();

//      copError2d.setToZero(soleFrame);
//      copError2d.sub(desiredCoP, centerOfPressure);
//
//      yoCoPErrorFiltered.update(copError2d);
//      yoCoPErrorPerpendicularToRotation.set(yoCoPErrorFiltered.cross(footAngularVelocityUnitVector));

      yoFootAngularVelocityFiltered.getFrameTuple2dIncludingFrame(footAngularVelocityUnitVector);
      footAngularVelocityUnitVector.normalize();

      yoCoRPositionFiltered.update(centerOfPressure);
      yoCoRPositionFiltered.getFrameTuple2dIncludingFrame(centerOfRotation);
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
         frameConvexPolygonWithLineIntersector2d.intersect(footPolygonInWorldFrame, lineOfRotationInWorldFrame);

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
   public void getLineOfRotation(FrameLine2d lineOfRotationToPack)
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
