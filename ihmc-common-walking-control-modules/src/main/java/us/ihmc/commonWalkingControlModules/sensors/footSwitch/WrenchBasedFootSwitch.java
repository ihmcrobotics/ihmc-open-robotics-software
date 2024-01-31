package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

//TODO Probably make an EdgeSwitch interface that has all the HeelSwitch and ToeSwitch stuff
public class WrenchBasedFootSwitch implements FootSwitchInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double MIN_FORCE_TO_COMPUTE_COP = 5.0;

   private final DoubleProvider contactForceThresholdLow;
   private final DoubleProvider contactForceThresholdHigh;
   private final DoubleProvider contactCoPThreshold;

   private final YoRegistry registry;

   private final ForceSensorDataReadOnly forceSensorData;

   private final YoBoolean isPastForceThresholdLow;
   private final GlitchFilteredYoBoolean isPastForceThresholdLowFiltered;
   private final YoBoolean isPastForceThresholdHigh;
   private final YoBoolean hasFootHitGround, isPastCoPThreshold;
   private final GlitchFilteredYoBoolean hasFootHitGroundFiltered;
   private final GlitchFilteredYoBoolean isPastCoPThresholdFiltered;

   private final YoDouble footForceMagnitude;
   private final YoDouble alphaFootLoadFiltering;
   private final AlphaFilteredYoVariable footLoadPercentage;

   private final Wrench footWrench;

   private final YoFramePoint2D centerOfPressure;
   private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
   private final ContactablePlaneBody contactablePlaneBody;
   private final double footLength;
   private final double footMinX;
   private final double footMaxX;

   private final YoFixedFrameSpatialVector yoFootForceTorque;
   private final YoFixedFrameSpatialVector yoFootForceTorqueInSole;
   private final YoFixedFrameSpatialVector yoFootForceTorqueInWorld;

   private final double robotTotalWeight;

   /**
    * @param namePrefix                prefix to use for naming the internal {@code YoVariable}s.
    * @param forceSensorData           the port to reading the sensor measurement.
    * @param robotTotalWeight          the robot weight used to compute the load distribution on this
    *                                  foot.
    * @param contactablePlaneBody      the contactable plane body of this foot, use to get the foot
    *                                  length and sole frame.
    * @param contactForceThresholdLow  the first force threshold. The foot is considered to have hit
    *                                  the ground if this threshold is met <b>and</b> the CoP threshold
    *                                  is met.
    * @param contactForceThresholdHigh the second force threshold. The foot is considered to have hit
    *                                  the ground if this threshold is met.
    * @param contactCoPThreshold       the center of pressure threshold. Expressed in percentage of
    *                                  foot length, this represents the margin away from the toe/heel
    *                                  line that the CoP needs to pass in order to consider that the
    *                                  foot has hit the ground.
    * @param yoGraphicsListRegistry
    * @param parentRegistry
    */
   public WrenchBasedFootSwitch(String namePrefix,
                                ForceSensorDataReadOnly forceSensorData,
                                double robotTotalWeight,
                                ContactablePlaneBody contactablePlaneBody,
                                DoubleProvider contactForceThresholdLow,
                                DoubleProvider contactForceThresholdHigh,
                                DoubleProvider contactCoPThreshold,
                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.forceSensorData = forceSensorData;
      this.robotTotalWeight = robotTotalWeight;
      this.contactablePlaneBody = contactablePlaneBody;
      this.contactForceThresholdLow = contactForceThresholdLow;
      this.contactForceThresholdHigh = contactForceThresholdHigh;
      this.contactCoPThreshold = contactCoPThreshold;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      ReferenceFrame measurementFrame = forceSensorData.getMeasurementFrame();
      ReferenceFrame soleFrame = contactablePlaneBody.getSoleFrame();
      yoFootForceTorque = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "Torque", measurementFrame, registry),
                                                        new YoFrameVector3D(namePrefix + "Force", measurementFrame, registry));
      yoFootForceTorqueInSole = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "TorqueSoleFrame", soleFrame, registry),
                                                              new YoFrameVector3D(namePrefix + "ForceSoleFrame", soleFrame, registry));
      yoFootForceTorqueInWorld = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "TorqueWorldFrame", worldFrame, registry),
                                                               new YoFrameVector3D(namePrefix + "ForceWorldFrame", worldFrame, registry));

      footForceMagnitude = new YoDouble(namePrefix + "FootForceMag", registry);

      isPastForceThresholdLow = new YoBoolean(namePrefix + "IsPastForceThresholdLow", registry);
      isPastForceThresholdLowFiltered = new GlitchFilteredYoBoolean(namePrefix + "IsPastForceThresholdLowFiltered", registry, isPastForceThresholdLow, 2);
      isPastForceThresholdHigh = new YoBoolean(namePrefix + "IsPastForceThresholdHigh", registry);
      isPastCoPThreshold = new YoBoolean(namePrefix + "IsPastCoPThreshold", registry);
      isPastCoPThresholdFiltered = new GlitchFilteredYoBoolean(namePrefix + "IsPastCoPThresholdFiltered", registry, isPastCoPThreshold, 3);

      hasFootHitGround = new YoBoolean(namePrefix + "FootHitGround", registry);
      // Final variable to identify if the foot has hit the ground
      hasFootHitGroundFiltered = new GlitchFilteredYoBoolean(namePrefix + "HasFootHitGroundFiltered", registry, hasFootHitGround, 2);

      alphaFootLoadFiltering = new YoDouble(namePrefix + "AlphaFootLoadFiltering", registry);
      alphaFootLoadFiltering.set(0.1);
      footLoadPercentage = new AlphaFilteredYoVariable(namePrefix + "FootLoadPercentage", registry, alphaFootLoadFiltering);

      centerOfPressure = new YoFramePoint2D(namePrefix + "CenterOfPressure", "", soleFrame, registry);

      footWrench = new Wrench(measurementFrame, (ReferenceFrame) null);

      footMinX = computeMinX(contactablePlaneBody);
      footMaxX = computeMaxX(contactablePlaneBody);
      footLength = computeLength(contactablePlaneBody);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      footWrench.setIncludingFrame(forceSensorData.getWrench());

      yoFootForceTorque.set(footWrench);
      yoFootForceTorqueInSole.setMatchingFrame(footWrench);
      yoFootForceTorqueInWorld.setMatchingFrame(footWrench);

      footForceMagnitude.set(footWrench.getLinearPart().norm());

      // Using the force in foot frame to ensure z is up when the foot is flat.
      // Sometimes the sensor can be mounted such that z is down.
      double forceZUp = yoFootForceTorqueInSole.getLinearPartZ();

      double fZPlus = MathTools.clamp(forceZUp, 0.0, Double.POSITIVE_INFINITY);
      footLoadPercentage.update(fZPlus / robotTotalWeight);

      isPastForceThresholdLow.set(forceZUp > contactForceThresholdLow.getValue());
      isPastForceThresholdLowFiltered.update();

      if (contactForceThresholdHigh != null)
      {
         isPastForceThresholdHigh.set(forceZUp > contactForceThresholdHigh.getValue());
      }
      else
      {
         isPastForceThresholdHigh.set(false);
      }

      // Computing Center of Pressure
      if (fZPlus < MIN_FORCE_TO_COMPUTE_COP)
         centerOfPressure.setToNaN();
      else
         copResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, footWrench, contactablePlaneBody.getSoleFrame());

      // Testing CoP threshold
      if (Double.isNaN(contactCoPThreshold.getValue()))
      {
         isPastCoPThreshold.set(true);
         isPastCoPThresholdFiltered.set(true);
      }
      else
      {
         /*
          * FIXME If we wanted to do the CoP filter properly, we should use make a ConvexPolygon2D from the
          * ContactablePlaneBody points and assert that the CoP is inside the polygon at a min distance from
          * the perimeter.
          */
         double copThreshold = contactCoPThreshold.getValue() * footLength;
         double minThresholdX = (footMinX + copThreshold);
         double maxThresholdX = (footMaxX - copThreshold);
         isPastCoPThreshold.set(centerOfPressure.getX() >= minThresholdX && centerOfPressure.getX() <= maxThresholdX);
         isPastCoPThresholdFiltered.update();
      }

      hasFootHitGround.set((isPastForceThresholdLowFiltered.getValue() && isPastCoPThresholdFiltered.getValue()) || isPastForceThresholdHigh.getValue());
      hasFootHitGroundFiltered.update();
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      return isPastForceThresholdLow.getBooleanValue();
   }

   @Override
   public boolean hasFootHitGroundFiltered()
   {
      return hasFootHitGroundFiltered.getValue();
   }

   @Override
   public void reset()
   {
      isPastCoPThresholdFiltered.set(false);
   }

   @Override
   public double getFootLoadPercentage()
   {
      return footLoadPercentage.getDoubleValue();
   }

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      return footWrench;
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return forceSensorData.getMeasurementFrame();
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return centerOfPressure;
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

   public ContactablePlaneBody getContactablePlaneBody()
   {
      return contactablePlaneBody;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
