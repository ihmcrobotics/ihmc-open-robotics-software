package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

//TODO Probably make an EdgeSwitch interface that has all the HeelSwitch and ToeSwitch stuff
public class WrenchBasedFootSwitch implements HeelSwitch, ToeSwitch
{
   private static final double MIN_FORCE_TO_COMPUTE_COP = 5.0;

   private final DoubleProvider contactThresholdForce;
   private final DoubleProvider secondContactThresholdForce;
   private final DoubleProvider footSwitchCoPThresholdFraction;

   private final YoVariableRegistry registry;

   private final ForceSensorDataReadOnly forceSensorData;

   private final YoBoolean isForceMagnitudePastThreshold;
   private final GlitchFilteredYoBoolean filteredIsForceMagnitudePastThreshold;
   private final YoBoolean isForceMagnitudePastSecondThreshold;
   private final YoBoolean hasFootHitGround, isCoPPastThreshold;
   private final YoBoolean trustFootSwitch, controllerDetectedTouchdown;
   private final GlitchFilteredYoBoolean filteredHasFootHitGround;

   private final YoDouble footForceMagnitude;
   private final YoDouble alphaFootLoadFiltering;
   private final AlphaFilteredYoVariable footLoadPercentage;

   private final Wrench footWrench;
   private final BagOfBalls footswitchCOPBagOfBalls;
   private final YoBoolean pastThreshold;
   private final YoBoolean heelHitGround;
   private final YoBoolean toeHitGround;
   private final GlitchFilteredYoBoolean pastThresholdFilter;
   private final GlitchFilteredYoBoolean heelHitGroundFilter;
   private final GlitchFilteredYoBoolean toeHitGroundFilter;

   private final YoFramePoint2D yoResolvedCoP;
   private final FramePoint2D resolvedCoP;
   private final FramePoint3D resolvedCoP3d = new FramePoint3D();
   private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
   private final ContactablePlaneBody contactablePlaneBody;
   private final double footLength;
   private final double footMinX;
   private final double footMaxX;
   private final FrameVector3D footForce = new FrameVector3D();
   private final FrameVector3D footTorque = new FrameVector3D();
   private final YoFrameVector3D yoFootForce;
   private final YoFrameVector3D yoFootTorque;
   private final YoFrameVector3D yoFootForceInFoot;
   private final YoFrameVector3D yoFootTorqueInFoot;
   private final YoFrameVector3D yoFootForceInWorld;
   private final YoFrameVector3D yoFootTorqueInWorld;

   private final double robotTotalWeight;

   private double minThresholdX;
   private double maxThresholdX;
   private final boolean showForceSensorFrames = false;
   private final YoGraphicReferenceFrame yoGraphicForceSensorMeasurementFrame, yoGraphicForceSensorFootFrame;

   private final AppearanceDefinition redAppearance = YoAppearance.Red();
   private final AppearanceDefinition blueAppearance = YoAppearance.Blue();

   public WrenchBasedFootSwitch(String namePrefix, ForceSensorDataReadOnly forceSensorData, double robotTotalWeight, ContactablePlaneBody contactablePlaneBody,
                                DoubleProvider contactThresholdForce, DoubleProvider secondContactThresholdForce, DoubleProvider footSwitchCoPThresholdFraction,
                                YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.contactThresholdForce = contactThresholdForce;
      this.secondContactThresholdForce = secondContactThresholdForce;
      this.footSwitchCoPThresholdFraction = footSwitchCoPThresholdFraction;

      yoFootForce = new YoFrameVector3D(namePrefix + "Force", forceSensorData.getMeasurementFrame(), registry);
      yoFootTorque = new YoFrameVector3D(namePrefix + "Torque", forceSensorData.getMeasurementFrame(), registry);
      yoFootForceInFoot = new YoFrameVector3D(namePrefix + "ForceFootFrame", contactablePlaneBody.getFrameAfterParentJoint(), registry);
      yoFootTorqueInFoot = new YoFrameVector3D(namePrefix + "TorqueFootFrame", contactablePlaneBody.getFrameAfterParentJoint(), registry);
      yoFootForceInWorld = new YoFrameVector3D(namePrefix + "ForceWorldFrame", ReferenceFrame.getWorldFrame(), registry);
      yoFootTorqueInWorld = new YoFrameVector3D(namePrefix + "TorqueWorldFrame", ReferenceFrame.getWorldFrame(), registry);

      if (showForceSensorFrames && yoGraphicsListRegistry != null)
      {
         final double scale = 1.0;
         yoGraphicForceSensorMeasurementFrame = new YoGraphicReferenceFrame(forceSensorData.getMeasurementFrame(), registry, false, .6 * scale,
               YoAppearance.Yellow());
         yoGraphicForceSensorFootFrame = new YoGraphicReferenceFrame(contactablePlaneBody.getFrameAfterParentJoint(), registry, false, scale,
               YoAppearance.AliceBlue());
         yoGraphicsListRegistry.registerYoGraphic(namePrefix + "MeasFrame", yoGraphicForceSensorMeasurementFrame);
         yoGraphicsListRegistry.registerYoGraphic(namePrefix + "FootFrame", yoGraphicForceSensorFootFrame);
      }
      else
      {
         yoGraphicForceSensorMeasurementFrame = null;
         yoGraphicForceSensorFootFrame = null;
      }

      footForceMagnitude = new YoDouble(namePrefix + "FootForceMag", registry);
      isForceMagnitudePastThreshold = new YoBoolean(namePrefix + "ForcePastThreshold", registry);
      hasFootHitGround = new YoBoolean(namePrefix + "FootHitGround", registry);

      trustFootSwitch = new YoBoolean(namePrefix + "TrustFootSwitch", registry);
      controllerDetectedTouchdown = new YoBoolean(namePrefix + "ControllerDetectedTouchdown", registry);
      trustFootSwitch.set(true);

      //TODO: Tune and triple check glitch filtering and timing of the virtual switches.
      filteredHasFootHitGround = new GlitchFilteredYoBoolean(namePrefix + "FilteredFootHitGround", registry, hasFootHitGround, 2);
      filteredIsForceMagnitudePastThreshold = new GlitchFilteredYoBoolean(namePrefix + "FilteredForcePastThresh", registry, isForceMagnitudePastThreshold, 2);
      isForceMagnitudePastSecondThreshold = new YoBoolean(namePrefix + "ForcePastSecondThresh", registry);
      isCoPPastThreshold = new YoBoolean(namePrefix + "CoPPastThresh", registry);

      this.robotTotalWeight = robotTotalWeight;
      this.alphaFootLoadFiltering = new YoDouble(namePrefix + "AlphaFootLoadFiltering", registry);
      alphaFootLoadFiltering.set(0.1);
      this.footLoadPercentage = new AlphaFilteredYoVariable(namePrefix + "FootLoadPercentage", registry, alphaFootLoadFiltering);

      double copVisualizerSize = 0.025;
      this.footswitchCOPBagOfBalls = new BagOfBalls(1, copVisualizerSize, namePrefix + "FootswitchCOP", registry, yoGraphicsListRegistry);

      this.pastThreshold = new YoBoolean(namePrefix + "PastFootswitchThreshold", registry);
      this.heelHitGround = new YoBoolean(namePrefix + "HeelHitGround", registry);
      this.toeHitGround = new YoBoolean(namePrefix + "ToeHitGround", registry);

      int filterWindowSize = 3;

      this.pastThresholdFilter = new GlitchFilteredYoBoolean(namePrefix + "PastFootswitchThresholdFilter", registry, pastThreshold, filterWindowSize);
      this.heelHitGroundFilter = new GlitchFilteredYoBoolean(namePrefix + "HeelHitGroundFilter", registry, heelHitGround, filterWindowSize);
      this.toeHitGroundFilter = new GlitchFilteredYoBoolean(namePrefix + "ToeHitGroundFilter", registry, toeHitGround, filterWindowSize);

      this.contactablePlaneBody = contactablePlaneBody;

      yoResolvedCoP = new YoFramePoint2D(namePrefix + "ResolvedCoP", "", contactablePlaneBody.getSoleFrame(), registry);
      resolvedCoP = new FramePoint2D(contactablePlaneBody.getSoleFrame());

      this.forceSensorData = forceSensorData;

      this.footWrench = new Wrench(forceSensorData.getMeasurementFrame(), null);

      this.footMinX = computeMinX(contactablePlaneBody);

      this.footMaxX = computeMaxX(contactablePlaneBody);
      this.footLength = computeLength(contactablePlaneBody);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasFootHitGround()
   {
      isForceMagnitudePastThreshold.set(isForceMagnitudePastThreshold());
      filteredIsForceMagnitudePastThreshold.update();

      if (secondContactThresholdForce != null)
      {
         isForceMagnitudePastSecondThreshold.set(yoFootForceInFoot.getZ() > secondContactThresholdForce.getValue());
      }
      else
      {
         isForceMagnitudePastSecondThreshold.set(false);
      }

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

   @Override
   public void reset()
   {
      pastThresholdFilter.set(false);
      heelHitGroundFilter.set(false);
      toeHitGroundFilter.set(false);
      controllerDetectedTouchdown.set(false);
   }

   @Override
   public void resetHeelSwitch()
   {
      heelHitGroundFilter.set(false);
   }

   @Override
   public void resetToeSwitch()
   {
      toeHitGroundFilter.set(false);
   }

   @Override
   public boolean hasHeelHitGround()
   {
      heelHitGround.set(isForceMagnitudePastThreshold());
      heelHitGroundFilter.update();
      return heelHitGroundFilter.getBooleanValue();
   }

   @Override
   public boolean hasToeHitGround()
   {
      toeHitGround.set(isForceMagnitudePastThreshold());
      toeHitGroundFilter.update();
      return toeHitGroundFilter.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      readSensorData(footWrench);

      footForce.setIncludingFrame(yoFootForceInFoot);

      footForce.clipToMinMax(0.0, Double.MAX_VALUE);

      footForceMagnitude.set(footForce.length());

      footLoadPercentage.update(footForce.getZ() / robotTotalWeight);

      return footLoadPercentage.getDoubleValue();
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      readSensorData(footWrenchToPack);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return forceSensorData.getMeasurementFrame();
   }

   private boolean isCoPPastThreshold()
   {
      if (Double.isNaN(footSwitchCoPThresholdFraction.getValue()))
         return true;

      updateCoP();

      minThresholdX = (footMinX + footSwitchCoPThresholdFraction.getValue() * footLength);
      maxThresholdX = (footMaxX - footSwitchCoPThresholdFraction.getValue() * footLength);

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

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      updateCoP();
      copToPack.setIncludingFrame(resolvedCoP);
   }

   @Override
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
         resolvedCoP3d.set(resolvedCoP);
         resolvedCoP3d.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   private Wrench footWrenchInBodyFixedFrame = new Wrench();

   private void readSensorData(Wrench footWrenchToPack)
   {
      forceSensorData.getWrench(footWrenchToPack);

      // First in measurement frame for all the frames...
      footForce.setToZero(footWrenchToPack.getReferenceFrame());
      footForce.set(footWrenchToPack.getLinearPart());
      yoFootForce.set(footForce);

      footTorque.setToZero(footWrenchToPack.getReferenceFrame());
      footTorque.set(footWrenchToPack.getAngularPart());
      yoFootTorque.set(footTorque);

      // magnitude of force part is independent of frame
      footForceMagnitude.set(footForce.length());

      // Now change to frame after the parent joint (ankle or wrist for example):
      footWrenchInBodyFixedFrame.setIncludingFrame(footWrenchToPack);
      footWrenchInBodyFixedFrame.changeFrame(contactablePlaneBody.getRigidBody().getBodyFixedFrame());

      footForce.setToZero(footWrenchInBodyFixedFrame.getReferenceFrame());
      footForce.set(footWrenchInBodyFixedFrame.getLinearPart());
      footTorque.setToZero(footWrenchInBodyFixedFrame.getReferenceFrame());
      footTorque.set(footWrenchInBodyFixedFrame.getAngularPart());

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
      return (yoFootForceInFoot.getZ() > contactThresholdForce.getValue());

      //      return (footFootMagnitude.getDoubleValue() > contactTresholdForce);
      //      return (footFootMagnitude.getDoubleValue() > MathTools.square(contactTresholdForce));
   }

   private static double computeLength(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector3D forward = new FrameVector3D(contactablePlaneBody.getSoleFrame(), 1.0, 0.0, 0.0);
      List<FramePoint3D> maxForward = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), forward, 1);

      FrameVector3D back = new FrameVector3D(contactablePlaneBody.getSoleFrame(), -1.0, 0.0, 0.0);
      List<FramePoint3D> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxForward.get(0).getX() - maxBack.get(0).getX();
   }

   private static double computeMinX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector3D back = new FrameVector3D(contactablePlaneBody.getSoleFrame(), -1.0, 0.0, 0.0);
      List<FramePoint3D> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxBack.get(0).getX();
   }

   private static double computeMaxX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector3D front = new FrameVector3D(contactablePlaneBody.getSoleFrame(), 1.0, 0.0, 0.0);
      List<FramePoint3D> maxFront = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), front, 1);

      return maxFront.get(0).getX();
   }

   @Override
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
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {
      this.trustFootSwitch.set(trustFootSwitch);
   }

   /**
    * Should use {@link #trustFootSwitchInSwing}.
    */
   @Deprecated
   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {
      throw new RuntimeException("This is not a different implementation by default.");
   }
}
