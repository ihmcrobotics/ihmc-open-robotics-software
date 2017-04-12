package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;

//TODO Probably make an EdgeSwitch interface that has all the HeelSwitch and ToeSwitch stuff
public class WrenchBasedFootSwitch implements HeelSwitch, ToeSwitch
{
   private static final double MIN_FORCE_TO_COMPUTE_COP = 5.0;

   private final DoubleYoVariable contactThresholdForce;
   private final DoubleYoVariable secondContactThresholdForce;

   private final YoVariableRegistry registry;

   private final ForceSensorDataReadOnly forceSensorData;
   private final DoubleYoVariable footSwitchCoPThresholdFraction;

   private final BooleanYoVariable isForceMagnitudePastThreshold;
   private final GlitchFilteredBooleanYoVariable filteredIsForceMagnitudePastThreshold;
   private final BooleanYoVariable isForceMagnitudePastSecondThreshold;
   private final BooleanYoVariable hasFootHitGround, isCoPPastThreshold;
   private final BooleanYoVariable trustFootSwitch, controllerDetectedTouchdown;
   private final GlitchFilteredBooleanYoVariable filteredHasFootHitGround;

   private final DoubleYoVariable footForceMagnitude;
   private final DoubleYoVariable alphaFootLoadFiltering;
   private final AlphaFilteredYoVariable footLoadPercentage;

   private final Wrench footWrench;
   private final BagOfBalls footswitchCOPBagOfBalls;
   private final BooleanYoVariable pastThreshold;
   private final BooleanYoVariable heelHitGround;
   private final BooleanYoVariable toeHitGround;
   private final GlitchFilteredBooleanYoVariable pastThresholdFilter;
   private final GlitchFilteredBooleanYoVariable heelHitGroundFilter;
   private final GlitchFilteredBooleanYoVariable toeHitGroundFilter;

   private final YoFramePoint2d yoResolvedCoP;
   private final FramePoint2d resolvedCoP;
   private final FramePoint resolvedCoP3d = new FramePoint();
   private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
   private final ContactablePlaneBody contactablePlaneBody;
   private final double footLength;
   private final double footMinX;
   private final double footMaxX;
   private final FrameVector footForce = new FrameVector();
   private final FrameVector footTorque = new FrameVector();
   private final YoFrameVector yoFootForce;
   private final YoFrameVector yoFootTorque;
   private final YoFrameVector yoFootForceInFoot;
   private final YoFrameVector yoFootTorqueInFoot;
   private final YoFrameVector yoFootForceInWorld;
   private final YoFrameVector yoFootTorqueInWorld;

   private final double robotTotalWeight;

   private double minThresholdX;
   private double maxThresholdX;
   private final boolean showForceSensorFrames = false;
   private final YoGraphicReferenceFrame yoGraphicForceSensorMeasurementFrame, yoGraphicForceSensorFootFrame;

   private final AppearanceDefinition redAppearance = YoAppearance.Red();
   private final AppearanceDefinition blueAppearance = YoAppearance.Blue();

   public WrenchBasedFootSwitch(String namePrefix, ForceSensorDataReadOnly forceSensorData, double footSwitchCoPThresholdFraction, double robotTotalWeight,
         ContactablePlaneBody contactablePlaneBody, YoGraphicsListRegistry yoGraphicsListRegistry, double contactThresholdForce,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.contactThresholdForce = new DoubleYoVariable(namePrefix + "ContactThresholdForce", registry);
      this.contactThresholdForce.set(contactThresholdForce);
      this.secondContactThresholdForce = new DoubleYoVariable(namePrefix + "SecondContactThresholdForce", registry);
      this.secondContactThresholdForce.set(Double.POSITIVE_INFINITY);

      yoFootForce = new YoFrameVector(namePrefix + "Force", forceSensorData.getMeasurementFrame(), registry);
      yoFootTorque = new YoFrameVector(namePrefix + "Torque", forceSensorData.getMeasurementFrame(), registry);
      yoFootForceInFoot = new YoFrameVector(namePrefix + "ForceFootFrame", contactablePlaneBody.getFrameAfterParentJoint(), registry);
      yoFootTorqueInFoot = new YoFrameVector(namePrefix + "TorqueFootFrame", contactablePlaneBody.getFrameAfterParentJoint(), registry);
      yoFootForceInWorld = new YoFrameVector(namePrefix + "ForceWorldFrame", ReferenceFrame.getWorldFrame(), registry);
      yoFootTorqueInWorld = new YoFrameVector(namePrefix + "TorqueWorldFrame", ReferenceFrame.getWorldFrame(), registry);

      if (showForceSensorFrames && yoGraphicsListRegistry != null)
      {
         final double scale = 1.0;
         yoGraphicForceSensorMeasurementFrame = new YoGraphicReferenceFrame(forceSensorData.getMeasurementFrame(), registry, .6 * scale,
               YoAppearance.Yellow());
         yoGraphicForceSensorFootFrame = new YoGraphicReferenceFrame(contactablePlaneBody.getFrameAfterParentJoint(), registry, scale,
               YoAppearance.AliceBlue());
         yoGraphicsListRegistry.registerYoGraphic(namePrefix + "MeasFrame", yoGraphicForceSensorMeasurementFrame);
         yoGraphicsListRegistry.registerYoGraphic(namePrefix + "FootFrame", yoGraphicForceSensorFootFrame);
      }
      else
      {
         yoGraphicForceSensorMeasurementFrame = null;
         yoGraphicForceSensorFootFrame = null;
      }

      footForceMagnitude = new DoubleYoVariable(namePrefix + "FootForceMag", registry);
      isForceMagnitudePastThreshold = new BooleanYoVariable(namePrefix + "ForcePastThreshold", registry);
      hasFootHitGround = new BooleanYoVariable(namePrefix + "FootHitGround", registry);

      trustFootSwitch = new BooleanYoVariable(namePrefix + "TrustFootSwitch", registry);
      controllerDetectedTouchdown = new BooleanYoVariable(namePrefix + "ControllerDetectedTouchdown", registry);
      trustFootSwitch.set(true);

      //TODO: Tune and triple check glitch filtering and timing of the virtual switches.
      filteredHasFootHitGround = new GlitchFilteredBooleanYoVariable(namePrefix + "FilteredFootHitGround", registry, hasFootHitGround, 1);
      filteredIsForceMagnitudePastThreshold = new GlitchFilteredBooleanYoVariable(namePrefix + "FilteredForcePastThresh", registry, isForceMagnitudePastThreshold, 2);
      isForceMagnitudePastSecondThreshold = new BooleanYoVariable(namePrefix + "ForcePastSecondThresh", registry);
      isCoPPastThreshold = new BooleanYoVariable(namePrefix + "CoPPastThresh", registry);

      this.robotTotalWeight = robotTotalWeight;
      this.alphaFootLoadFiltering = new DoubleYoVariable(namePrefix + "AlphaFootLoadFiltering", registry);
      alphaFootLoadFiltering.set(0.5);
      this.footLoadPercentage = new AlphaFilteredYoVariable(namePrefix + "FootLoadPercentage", registry, alphaFootLoadFiltering);

      double copVisualizerSize = 0.025;
      this.footswitchCOPBagOfBalls = new BagOfBalls(1, copVisualizerSize, namePrefix + "FootswitchCOP", registry, yoGraphicsListRegistry);

      this.pastThreshold = new BooleanYoVariable(namePrefix + "PastFootswitchThreshold", registry);
      this.heelHitGround = new BooleanYoVariable(namePrefix + "HeelHitGround", registry);
      this.toeHitGround = new BooleanYoVariable(namePrefix + "ToeHitGround", registry);

      int filterWindowSize = 3;

      this.pastThresholdFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "PastFootswitchThresholdFilter", registry, pastThreshold, filterWindowSize);
      this.heelHitGroundFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "HeelHitGroundFilter", registry, heelHitGround, filterWindowSize);
      this.toeHitGroundFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "ToeHitGroundFilter", registry, toeHitGround, filterWindowSize);

      this.contactablePlaneBody = contactablePlaneBody;

      yoResolvedCoP = new YoFramePoint2d(namePrefix + "ResolvedCoP", "", contactablePlaneBody.getSoleFrame(), registry);
      resolvedCoP = new FramePoint2d(contactablePlaneBody.getSoleFrame());

      this.forceSensorData = forceSensorData;
      this.footSwitchCoPThresholdFraction = new DoubleYoVariable(namePrefix + "footSwitchCoPThresholdFraction", registry);
      this.footSwitchCoPThresholdFraction.set(footSwitchCoPThresholdFraction);

      this.footWrench = new Wrench(forceSensorData.getMeasurementFrame(), null);

      this.footMinX = computeMinX(contactablePlaneBody);

      this.footMaxX = computeMaxX(contactablePlaneBody);
      this.footLength = computeLength(contactablePlaneBody);

      parentRegistry.addChild(registry);
   }

   public void setSecondContactThresholdForce(double secondContactThresholdForce)
   {
      this.secondContactThresholdForce.set(secondContactThresholdForce);
   }

   public boolean hasFootHitGround()
   {
      isForceMagnitudePastThreshold.set(isForceMagnitudePastThreshold());
      filteredIsForceMagnitudePastThreshold.update();

      isForceMagnitudePastSecondThreshold.set(yoFootForceInFoot.getZ() > secondContactThresholdForce.getDoubleValue());
      isCoPPastThreshold.set(isCoPPastThreshold());

      hasFootHitGround.set(
            (filteredIsForceMagnitudePastThreshold.getBooleanValue() && isCoPPastThreshold.getBooleanValue()) ||
                  isForceMagnitudePastSecondThreshold.getBooleanValue());
      //      hasFootHitGround.set(isForceMagnitudePastThreshold.getBooleanValue());
      filteredHasFootHitGround.update();

      if (trustFootSwitch.getBooleanValue())
         return filteredHasFootHitGround.getBooleanValue();
      else
         return controllerDetectedTouchdown.getBooleanValue();
   }

   public void reset()
   {
      pastThresholdFilter.set(false);
      heelHitGroundFilter.set(false);
      toeHitGroundFilter.set(false);
      controllerDetectedTouchdown.set(false);
   }

   public void resetHeelSwitch()
   {
      heelHitGroundFilter.set(false);
   }

   public void resetToeSwitch()
   {
      toeHitGroundFilter.set(false);
   }

   public boolean hasHeelHitGround()
   {
      heelHitGround.set(isForceMagnitudePastThreshold());
      heelHitGroundFilter.update();
      return heelHitGroundFilter.getBooleanValue();
   }

   public boolean hasToeHitGround()
   {
      toeHitGround.set(isForceMagnitudePastThreshold());
      toeHitGroundFilter.update();
      return toeHitGroundFilter.getBooleanValue();
   }

   public double computeFootLoadPercentage()
   {
      readSensorData(footWrench);

      yoFootForceInFoot.getFrameTupleIncludingFrame(footForce);

      footForce.clipToMinMax(0.0, Double.MAX_VALUE);

      footForceMagnitude.set(footForce.length());

      footLoadPercentage.update(footForce.getZ() / robotTotalWeight);

      return footLoadPercentage.getDoubleValue();
   }

   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      readSensorData(footWrenchToPack);
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return forceSensorData.getMeasurementFrame();
   }

   private boolean isCoPPastThreshold()
   {
      if (Double.isNaN(footSwitchCoPThresholdFraction.getDoubleValue()))
         return true;

      updateCoP();

      minThresholdX = (footMinX + footSwitchCoPThresholdFraction.getDoubleValue() * footLength);
      maxThresholdX = (footMaxX - footSwitchCoPThresholdFraction.getDoubleValue() * footLength);

      if (toeHitGroundFilter.getBooleanValue())
         pastThreshold.set(resolvedCoP.getX() <= maxThresholdX);
      else if (heelHitGroundFilter.getBooleanValue())
         pastThreshold.set(resolvedCoP.getX() >= minThresholdX);
      else
         pastThreshold.set(resolvedCoP.getX() >= minThresholdX && resolvedCoP.getX() <= maxThresholdX);

      pastThresholdFilter.update();

      AppearanceDefinition appearanceDefinition = pastThresholdFilter.getBooleanValue() ? redAppearance : blueAppearance;

      footswitchCOPBagOfBalls.setBall(resolvedCoP3d, appearanceDefinition, 0);

      return pastThresholdFilter.getBooleanValue();
   }

   public void computeAndPackCoP(FramePoint2d copToPack)
   {
      updateCoP();
      copToPack.setIncludingFrame(resolvedCoP);
   }

   public void updateCoP()
   {
      readSensorData(footWrench);

      if (Math.abs(footWrench.getLinearPartZ()) < MIN_FORCE_TO_COMPUTE_COP)
      {
         yoResolvedCoP.setToNaN();
         resolvedCoP3d.setToNaN(ReferenceFrame.getWorldFrame());
         resolvedCoP.setToNaN();
      }
      else
      {
         copResolver.resolveCenterOfPressureAndNormalTorque(resolvedCoP, footWrench, contactablePlaneBody.getSoleFrame());
         yoResolvedCoP.set(resolvedCoP);
         
         resolvedCoP3d.setToZero(resolvedCoP.getReferenceFrame());
         resolvedCoP3d.setXY(resolvedCoP);
         resolvedCoP3d.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   private Wrench footWrenchInBodyFixedFrame = new Wrench();

   private void readSensorData(Wrench footWrenchToPack)
   {
      forceSensorData.getWrench(footWrenchToPack);

      // First in measurement frame for all the frames...
      footForce.setToZero(footWrenchToPack.getExpressedInFrame());
      footWrenchToPack.getLinearPart(footForce);
      yoFootForce.set(footForce);

      footTorque.setToZero(footWrenchToPack.getExpressedInFrame());
      footWrenchToPack.getAngularPart(footTorque);
      yoFootTorque.set(footTorque);

      // magnitude of force part is independent of frame
      footForceMagnitude.set(footForce.length());

      // Now change to frame after the parent joint (ankle or wrist for example):
      footWrenchInBodyFixedFrame.set(footWrenchToPack);
      footWrenchInBodyFixedFrame.changeFrame(contactablePlaneBody.getRigidBody().getBodyFixedFrame());

      footForce.setToZero(footWrenchInBodyFixedFrame.getExpressedInFrame());
      footWrenchInBodyFixedFrame.getLinearPart(footForce);
      footTorque.setToZero(footWrenchInBodyFixedFrame.getExpressedInFrame());
      footWrenchInBodyFixedFrame.getAngularPart(footTorque);

      footForce.changeFrame(contactablePlaneBody.getFrameAfterParentJoint());
      yoFootForceInFoot.set(footForce);

      footTorque.changeFrame(contactablePlaneBody.getFrameAfterParentJoint());
      yoFootTorqueInFoot.set(footTorque);

      footForce.changeFrame(ReferenceFrame.getWorldFrame());
      footTorque.changeFrame(ReferenceFrame.getWorldFrame());

      yoFootForceInWorld.set(footForce);
      yoFootTorqueInWorld.set(footTorque);

      updateSensorVisualizer();
   }

   private void updateSensorVisualizer()
   {
      if (yoGraphicForceSensorMeasurementFrame != null)
         yoGraphicForceSensorMeasurementFrame.update();
      if (yoGraphicForceSensorFootFrame != null)
         yoGraphicForceSensorFootFrame.update();
   }

   private boolean isForceMagnitudePastThreshold()
   {
      readSensorData(footWrench);

      //TODO: We switched to just z and made sure it was positive.
      //TODO: Check which reference frames all this stuff is in.
      //TODO: Make SCS and Gazebo consistent if possible.
      return (yoFootForceInFoot.getZ() > contactThresholdForce.getDoubleValue());

      //      return (footFootMagnitude.getDoubleValue() > contactTresholdForce);
      //      return (footFootMagnitude.getDoubleValue() > MathTools.square(contactTresholdForce));
   }

   private static double computeLength(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector forward = new FrameVector(contactablePlaneBody.getSoleFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> maxForward = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), forward, 1);

      FrameVector back = new FrameVector(contactablePlaneBody.getSoleFrame(), -1.0, 0.0, 0.0);
      List<FramePoint> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxForward.get(0).getX() - maxBack.get(0).getX();
   }

   private static double computeMinX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector back = new FrameVector(contactablePlaneBody.getSoleFrame(), -1.0, 0.0, 0.0);
      List<FramePoint> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxBack.get(0).getX();
   }

   private static double computeMaxX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector front = new FrameVector(contactablePlaneBody.getSoleFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> maxFront = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), front, 1);

      return maxFront.get(0).getX();
   }

   public boolean getForceMagnitudePastThreshhold()
   {
      return isForceMagnitudePastThreshold.getBooleanValue();
   }

   public ContactablePlaneBody getContactablePlaneBody()
   {
      return contactablePlaneBody;
   }

   @Override
   @Deprecated
   public void setFootContactState(boolean hasFootHitGround)
   {
      controllerDetectedTouchdown.set(hasFootHitGround);
   }

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {
      this.trustFootSwitch.set(trustFootSwitch);
   }
}
