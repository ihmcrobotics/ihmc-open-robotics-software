package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.plotting.Artifact;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.yoUtilities.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

/**
 * The ScrewAxisCalculator is a tool to compute the instantaneous axis of rotation of a rigid body.
 * @author Sylvain
 *
 */
public class FootRotationCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry;

   private final YoFrameVector2d yoFootAngularVelocity;
   private final DoubleYoVariable angularVelocityAroundLineOfRotation;
   private final DoubleYoVariable alphaFilter;
   private final AlphaFilteredYoFrameVector2d yoAngularVelocityFiltered;
   private final AlphaFilteredYoFramePoint2d yoCenterOfRotationFiltered;
   private final YoFrameVector yoScrewVector;
   private final YoFrameLineSegment2d yoLineOfRotation;
   private final DoubleYoVariable angularVelocityOfLineOfRotation;
   private final DoubleYoVariable footAngularVelocityThreshold;
   
   private final DoubleYoVariable stableAngularVelocityThreshold;
   private final BooleanYoVariable isLineOfRotationStable;
   private final BooleanYoVariable isFootAngularVelocityPastThreshold;
   private final GlitchFilteredBooleanYoVariable isLineOfRotationStableFiltered;

   private final BooleanYoVariable isFootRotating;
   
   private final FrameVector screwAxis = new FrameVector();
   private final FrameVector2d screwAxis2d = new FrameVector2d();
   private final FrameVector2d screwAxis2dPrevValue = new FrameVector2d();

   private final FramePoint2d centerOfRotation = new FramePoint2d();
   private final FrameLine2d lineOfRotationInSoleFrame = new FrameLine2d();
   private final FrameLine2d lineOfRotationInWorldFrame = new FrameLine2d();
   private final FrameLineSegment2d lineSegmentOfRotation = new FrameLineSegment2d();

   private final ContactablePlaneBody rotatingBody;
   private final TwistCalculator twistCalculator;

   private final ReferenceFrame soleFrame;
   
   private final Twist bodyTwist = new Twist();
   private final FrameConvexPolygon2d footPolygonInSoleFrame = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d footPolygonInWorldFrame = new FrameConvexPolygon2d();
   private final Artifact lineOfRotationArtifact;
   private final double dt;

   public FootRotationCalculator(String namePrefix, double dt, ContactablePlaneBody rotatingFoot, TwistCalculator twistCalculator, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.twistCalculator = twistCalculator;
      this.rotatingBody = rotatingFoot;
      this.soleFrame = rotatingFoot.getSoleFrame();
      this.dt = dt;

      footPolygonInSoleFrame.setIncludingFrameAndUpdate(rotatingFoot.getContactPoints2d());
      
      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      yoFootAngularVelocity = new YoFrameVector2d(namePrefix + "AngularVelocity", soleFrame, registry);
      alphaFilter = new DoubleYoVariable(namePrefix + name + "AlphaFilter", registry);
      yoAngularVelocityFiltered = AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d(namePrefix + "AngularVelocityFiltered", "", registry, alphaFilter, yoFootAngularVelocity);
      yoCenterOfRotationFiltered = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d(namePrefix + "CenterOfRotationFiltered", "", registry, alphaFilter, soleFrame);
      yoScrewVector = new YoFrameVector(namePrefix + "ScrewVector", worldFrame, registry);
      yoLineOfRotation = new YoFrameLineSegment2d(namePrefix + "LineOfRotation", "", worldFrame, registry);
      angularVelocityOfLineOfRotation = new DoubleYoVariable(namePrefix + "AngularVelocityOfLineOfRotation", registry);
      
      angularVelocityAroundLineOfRotation = new DoubleYoVariable(namePrefix + "AngularVelocityAroundLineOfRotation", registry);

      stableAngularVelocityThreshold = new DoubleYoVariable(namePrefix + "StableAngularVelocityThreshold", registry);
      stableAngularVelocityThreshold.set(1.0);
      isFootAngularVelocityPastThreshold = new BooleanYoVariable(namePrefix + "IsFootAngularVelocityPastThreshold", registry);
      isLineOfRotationStable = new BooleanYoVariable(namePrefix + "IsLineOfRotationStable", registry);
      isLineOfRotationStableFiltered = new GlitchFilteredBooleanYoVariable(namePrefix + "IsLineOfRotationStableFiltered", registry, isLineOfRotationStable, 3);

      isFootRotating = new BooleanYoVariable(namePrefix + "IsFootRotating", registry);

      footAngularVelocityThreshold = new DoubleYoVariable(namePrefix + "FootAngularVelocityThreshold", registry);
      footAngularVelocityThreshold.set(0.5);
      screwAxis2d.setToZero(soleFrame);
      screwAxis2dPrevValue.setToZero(soleFrame);
      lineOfRotationInSoleFrame.setIncludingFrame(soleFrame, 0.0, 0.0, 1.0, 0.0);
      
      if (yoGraphicsListRegistry != null)
      {
         lineOfRotationArtifact = new YoArtifactLineSegment2d(namePrefix + "LineOfRotation", yoLineOfRotation, Color.ORANGE);
         yoGraphicsListRegistry.registerArtifact("FootRotation", lineOfRotationArtifact);
      }
      else
      {
         lineOfRotationArtifact = null;
      }
   }

   private final FramePoint2d tempFramePoint2d = new FramePoint2d();
   
   public void compute(FramePoint2d centerOfPressureInSoleFrame)
   {
      footPolygonInWorldFrame.setIncludingFrameAndUpdate(footPolygonInSoleFrame);
      footPolygonInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);

      twistCalculator.packTwistOfBody(bodyTwist, rotatingBody.getRigidBody());
      bodyTwist.packAngularPart(screwAxis);
      screwAxis.changeFrame(soleFrame);
      screwAxis.setZ(0.0);
      screwAxis2d.setIncludingFrame(soleFrame, screwAxis.getX(), screwAxis.getY());
      yoFootAngularVelocity.set(screwAxis2d);
      yoAngularVelocityFiltered.update();
      yoAngularVelocityFiltered.getFrameTuple2dIncludingFrame(screwAxis2d);
      angularVelocityOfLineOfRotation.set(screwAxis2d.angle(screwAxis2dPrevValue) / dt);
      screwAxis2dPrevValue.setIncludingFrame(screwAxis2d);

      screwAxis.changeFrame(worldFrame);
      screwAxis.normalize();
      yoScrewVector.set(screwAxis);

      yoCenterOfRotationFiltered.update(centerOfPressureInSoleFrame);
      yoCenterOfRotationFiltered.getFrameTuple2dIncludingFrame(centerOfRotation);
      
      isLineOfRotationStable.set(Math.abs(angularVelocityOfLineOfRotation.getDoubleValue()) < stableAngularVelocityThreshold.getDoubleValue());
      isLineOfRotationStableFiltered.update();
      
      screwAxis2d.normalize();
      angularVelocityAroundLineOfRotation.set(yoAngularVelocityFiltered.dot(screwAxis2d));
      isFootAngularVelocityPastThreshold.set(angularVelocityAroundLineOfRotation.getDoubleValue() > footAngularVelocityThreshold.getDoubleValue());
      
      isFootRotating.set(isLineOfRotationStableFiltered.getBooleanValue() && isFootAngularVelocityPastThreshold.getBooleanValue());
      
      if (isFootRotating.getBooleanValue())
      {
         tempFramePoint2d.setIncludingFrame(centerOfRotation);
         tempFramePoint2d.scaleAdd(-10000.0, screwAxis2d, centerOfRotation);
         lineOfRotationInSoleFrame.set(tempFramePoint2d, screwAxis2d);
         lineOfRotationInWorldFrame.setIncludingFrame(lineOfRotationInSoleFrame);
         lineOfRotationInWorldFrame.changeFrameAndProjectToXYPlane(worldFrame);
         FramePoint2d[] intersections = footPolygonInWorldFrame.intersectionWith(lineOfRotationInWorldFrame);

         if (intersections == null || intersections.length == 1)
         {
            yoLineOfRotation.setToNaN();
         }
         else
         {
            lineSegmentOfRotation.setIncludingFrame(intersections);
            yoLineOfRotation.setFrameLineSegment2d(lineSegmentOfRotation);
         }
      }
      else
      {
         yoLineOfRotation.setToNaN();
      }
   }

   public boolean isFootRotating()
   {
      return isFootRotating.getBooleanValue();
   }

   public void getLineOfRotation(FrameLine2d lineOfRotationToPack)
   {
      lineOfRotationToPack.setIncludingFrame(lineOfRotationInSoleFrame);
   }

   public void reset()
   {
      yoLineOfRotation.setToNaN();
      yoAngularVelocityFiltered.reset();
      yoAngularVelocityFiltered.setToZero();
      yoCenterOfRotationFiltered.reset();
      yoCenterOfRotationFiltered.setToZero();
      angularVelocityOfLineOfRotation.set(Double.NaN);
      angularVelocityAroundLineOfRotation.set(Double.NaN);
      isLineOfRotationStable.set(false);
      isLineOfRotationStableFiltered.set(false);
      isFootRotating.set(false);
   }

   public double getAlphaFilter()
   {
      return alphaFilter.getDoubleValue();
   }

   public void setAlphaFilter(double alpha)
   {
      alphaFilter.set(alpha);
   }

   public double getFootAngularVelocityThreshold()
   {
      return footAngularVelocityThreshold.getDoubleValue();
   }

   public void setFootAngularVelocityThreshold(double threshold)
   {
      footAngularVelocityThreshold.set(threshold);
   }

   public double getStableAngularVelocityThreshold()
   {
      return stableAngularVelocityThreshold.getDoubleValue();
   }

   public void setStableAngularVelocityThreshold(double threshold)
   {
      stableAngularVelocityThreshold.set(threshold);
   }
}
