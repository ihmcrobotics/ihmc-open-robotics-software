package us.ihmc.commonWalkingControlModules.sensors;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.GlitchFilteredBooleanYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

//TODO Probably make an EdgeSwitch interface that has all the HeelSwitch and ToeSwitch stuff
public class WrenchBasedFootSwitch implements HeelSwitch, ToeSwitch
{
   private final DoubleYoVariable contactThresholdForce;

   private final YoVariableRegistry registry;

   private final ForceSensorData forceSensorData;
   private final DoubleYoVariable footSwitchCoPThresholdFraction;

   private final GlitchFilteredBooleanYoVariable isForceMagnitudePastThreshold;
   private final BooleanYoVariable hasFootHitGround, isCoPPastThreshold;
   private final GlitchFilteredBooleanYoVariable filteredHasFootHitGround;

   private final DoubleYoVariable footForceMagnitude;
   private final DoubleYoVariable alphaFootLoadFiltering;
   private final AlphaFilteredYoVariable footLoadPercentage;

   private final Wrench footWrench;
   private final BagOfBalls footswitchCOPBagOfBalls;
   private final BooleanYoVariable pastThreshold;
   private final BooleanYoVariable heelHitGround;
   private final BooleanYoVariable toeHitGround;
   private final BooleanYoVariable swingTrajectoryWasReplanned;
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

   private final double robotTotalWeight;

   public WrenchBasedFootSwitch(String namePrefix, ForceSensorData forceSensorData, double footSwitchCoPThresholdFraction, double robotTotalWeight, ContactablePlaneBody contactablePlaneBody,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, double contactThresholdForce, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.contactThresholdForce = new DoubleYoVariable(namePrefix + "ContactThresholdForce", registry);
      this.contactThresholdForce.set(contactThresholdForce); 


      yoFootForce = new YoFrameVector(namePrefix + "FootForce", forceSensorData.getMeasurementFrame(), registry);
      yoFootTorque = new YoFrameVector(namePrefix + "FootTorque", forceSensorData.getMeasurementFrame(), registry);
      yoFootForceInFoot = new YoFrameVector(namePrefix + "FootForceInFootFrame", contactablePlaneBody.getBodyFrame(), registry);
      yoFootTorqueInFoot = new YoFrameVector(namePrefix + "FootTorqueInFootFrame", contactablePlaneBody.getBodyFrame(), registry);

      footForceMagnitude = new DoubleYoVariable(namePrefix + "FootForceMag", registry);
      hasFootHitGround = new BooleanYoVariable(namePrefix + "FootHitGround", registry);

      //TODO: Tune and triple check glitch filtering and timing of the virtual switches.
      filteredHasFootHitGround = new GlitchFilteredBooleanYoVariable(namePrefix + "FilteredFootHitGround", registry, hasFootHitGround, 1); 
      isForceMagnitudePastThreshold = new GlitchFilteredBooleanYoVariable(namePrefix + "ForcePastThresh", registry, 2);
      isCoPPastThreshold = new BooleanYoVariable(namePrefix + "CoPPastThresh", registry);

      this.robotTotalWeight = robotTotalWeight;
      this.alphaFootLoadFiltering = new DoubleYoVariable(namePrefix + "AlphaFootLoadFiltering", registry);
      alphaFootLoadFiltering.set(0.5);
      this.footLoadPercentage = new AlphaFilteredYoVariable(namePrefix + "FootLoadPercentage", registry, alphaFootLoadFiltering);

      this.swingTrajectoryWasReplanned = new BooleanYoVariable(namePrefix + "SwingFootTrajectoryWasReplanned", registry);
      this.swingTrajectoryWasReplanned.set(false);

      double copVisualizerSize = 0.025;
      this.footswitchCOPBagOfBalls = new BagOfBalls(1, copVisualizerSize, namePrefix + "FootswitchCOP", registry,
            dynamicGraphicObjectsListRegistry);

      this.pastThreshold = new BooleanYoVariable(namePrefix + "PastFootswitchThresold", registry);
      this.heelHitGround = new BooleanYoVariable(namePrefix + "HeelHitGround", registry);
      this.toeHitGround = new BooleanYoVariable(namePrefix + "ToeHitGround", registry);
      
      int filterWindowSize = 3;
      
      this.pastThresholdFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "PastFootswitchThresholdFilter", registry, pastThreshold, filterWindowSize);
      this.heelHitGroundFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "HeelHitGroundFilter", registry, heelHitGround, filterWindowSize);
      this.toeHitGroundFilter = new GlitchFilteredBooleanYoVariable(namePrefix + "ToeHitGroundFilter", registry, toeHitGround, filterWindowSize);

      this.contactablePlaneBody = contactablePlaneBody;

      yoResolvedCoP = new YoFramePoint2d(namePrefix + "ResolvedCoP", "", contactablePlaneBody.getPlaneFrame(), registry);
      resolvedCoP = new FramePoint2d(contactablePlaneBody.getPlaneFrame());

      this.forceSensorData = forceSensorData;
      this.footSwitchCoPThresholdFraction = new DoubleYoVariable(namePrefix + "footSwitchCoPThresholdFraction", registry);
      this.footSwitchCoPThresholdFraction.set(footSwitchCoPThresholdFraction);

      this.footWrench = new Wrench(forceSensorData.getMeasurementFrame(), null);

      this.footMinX = computeMinX(contactablePlaneBody);
      this.footMaxX = computeMaxX(contactablePlaneBody);
      this.footLength = computeLength(contactablePlaneBody);

      parentRegistry.addChild(registry);
   }

   public boolean hasFootHitGround()
   {      
      isForceMagnitudePastThreshold.set(isForceMagnitudePastThreshold());
      isCoPPastThreshold.set(isCoPPastThreshold());

      // If you have had to replan your swing foot trajectory due to a push, don't worry about cop being past threshold, just switch on touchdown
      if(swingTrajectoryWasReplanned.getBooleanValue())
      {
    	  hasFootHitGround.set(isForceMagnitudePastThreshold.getBooleanValue());
      }
      else
      {
    	  hasFootHitGround.set(isForceMagnitudePastThreshold.getBooleanValue() && isCoPPastThreshold.getBooleanValue());
      }
      
//      hasFootHitGround.set(isForceMagnitudePastThreshold.getBooleanValue());
      filteredHasFootHitGround.update();
      
      return filteredHasFootHitGround.getBooleanValue();
   }

   public void reset()
   {
      pastThresholdFilter.set(false);
      heelHitGroundFilter.set(false);
      toeHitGroundFilter.set(false);
      swingTrajectoryWasReplanned.set(false);
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

   private boolean isCoPPastThreshold()
   {
      updateCoP();

      double minThresholdX = footMinX + footSwitchCoPThresholdFraction.getDoubleValue() * footLength;
      double maxThresholdX = footMaxX - footSwitchCoPThresholdFraction.getDoubleValue() * footLength;

      if (toeHitGroundFilter.getBooleanValue())
         pastThreshold.set(resolvedCoP.getX() <= maxThresholdX);
      else if (heelHitGroundFilter.getBooleanValue())
         pastThreshold.set(resolvedCoP.getX() >= minThresholdX);
      else
         pastThreshold.set(resolvedCoP.getX() >= minThresholdX && resolvedCoP.getX() <= maxThresholdX);

      pastThresholdFilter.update();

      AppearanceDefinition appearanceDefinition = pastThresholdFilter.getBooleanValue() ? YoAppearance.Red() : YoAppearance.Blue();

      footswitchCOPBagOfBalls.setBall(resolvedCoP3d, appearanceDefinition, 0);

      return pastThresholdFilter.getBooleanValue();
   }

   public void computeAndPackCoP(FramePoint2d copToPack)
   {
      updateCoP();
      copToPack.setIncludingFrame(resolvedCoP);
   }
   
   public void setSwingTrajectoryWasReplanned(boolean value)
   {
	   swingTrajectoryWasReplanned.set(value);
   }

   public void updateCoP()
   {
      readSensorData(footWrench);
      copResolver.resolveCenterOfPressureAndNormalTorque(resolvedCoP, footWrench, contactablePlaneBody.getPlaneFrame());
      yoResolvedCoP.set(resolvedCoP);

      resolvedCoP3d.setToZero(resolvedCoP.getReferenceFrame());
      resolvedCoP3d.setXY(resolvedCoP);
      resolvedCoP3d.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void readSensorData(Wrench footWrench)
   {
      forceSensorData.packWrench(footWrench);

      footForce.setToZero(footWrench.getExpressedInFrame());
      footWrench.packLinearPart(footForce);
      yoFootForce.set(footForce);
      footForce.changeFrame(contactablePlaneBody.getBodyFrame());
      yoFootForceInFoot.set(footForce);
      footForceMagnitude.set(footForce.length());

      // magnitude of force part is independent of frame
      footForceMagnitude.set(footForce.length());

      footTorque.setToZero(footWrench.getExpressedInFrame());
      footWrench.packAngularPart(footTorque);
      yoFootTorque.set(footTorque);
      footTorque.changeFrame(contactablePlaneBody.getBodyFrame());
      yoFootTorqueInFoot.set(footTorque);

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
      FrameVector forward = new FrameVector(contactablePlaneBody.getPlaneFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> maxForward = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), forward, 1);

      FrameVector back = new FrameVector(contactablePlaneBody.getPlaneFrame(), -1.0, 0.0, 0.0);
      List<FramePoint> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxForward.get(0).getX() - maxBack.get(0).getX();
   }

   private static double computeMinX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector back = new FrameVector(contactablePlaneBody.getPlaneFrame(), -1.0, 0.0, 0.0);
      List<FramePoint> maxBack = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), back, 1);

      return maxBack.get(0).getX();
   }

   private static double computeMaxX(ContactablePlaneBody contactablePlaneBody)
   {
      FrameVector front = new FrameVector(contactablePlaneBody.getPlaneFrame(), 1.0, 0.0, 0.0);
      List<FramePoint> maxFront = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactablePlaneBody.getContactPointsCopy(), front, 1);

      return maxFront.get(0).getX();
   }
}
