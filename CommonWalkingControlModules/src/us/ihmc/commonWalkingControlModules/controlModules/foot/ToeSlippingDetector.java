package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class ToeSlippingDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable alpha;
   private final AlphaFilteredYoVariable toeForceFiltered;
   private final AlphaFilteredYoFrameVector toeLinearVelocityFiltered;

   private final YoFramePoint initialToePosition;
   private final DoubleYoVariable toeSlippageDistance;

   private final DoubleYoVariable forceMagnitudeThreshold;
   private final DoubleYoVariable velocityThreshold;
   private final DoubleYoVariable slippageDistanceThreshold;

   private final BooleanYoVariable isToeSlipping;

   private final double dt;
   private final RigidBody foot;
   private final TwistCalculator twistCalculator;
   private final FootSwitchInterface footSwitch;

   public ToeSlippingDetector(String namePrefix, double controlDT, RigidBody foot, TwistCalculator twistCalculator, FootSwitchInterface footSwitch,
                              YoVariableRegistry parentRegistry)
   {
      dt = controlDT;
      this.foot = foot;
      this.twistCalculator = twistCalculator;
      this.footSwitch = footSwitch;
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      alpha = new DoubleYoVariable(namePrefix + "AlphaFilter", registry);
      toeForceFiltered = new AlphaFilteredYoVariable(namePrefix + "ToeForceFiltered", "", registry, alpha);
      toeLinearVelocityFiltered = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "ToeLinearVelocityFiltered", "", registry, alpha,
                                                                                              worldFrame);

      initialToePosition = new YoFramePoint(namePrefix + "ToeInitial", worldFrame, registry);
      toeSlippageDistance = new DoubleYoVariable(namePrefix + "ToeSlippageDistance", registry);

      forceMagnitudeThreshold = new DoubleYoVariable(namePrefix + "ForceMagnitudeThreshold", registry);
      velocityThreshold = new DoubleYoVariable(namePrefix + "VelocityThreshold", registry);
      slippageDistanceThreshold = new DoubleYoVariable(namePrefix + "SlippageDistanceThreshold", registry);

      isToeSlipping = new BooleanYoVariable(namePrefix + "IsToeSlipping", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Configure the parameters for this module.
    * <p>
    * The {@code ToeSlippingDetector} is meant to detect if the toe of the support performing
    * toe-off is strongly slipping. When this is the case, the walking controller will attempt to
    * stop the toe-off by swinging the foot.
    * </p>
    * 
    * @param forceMagnitudeThreshold as long as the foot force magnitude remains above this
    *           threshold, the toe will be considered as not slipping.
    * @param velocityThreshold this is one of the condition to trigger the slipping detection: the
    *           toe linear velocity magnitude has to be greater than this threshold.
    * @param slippageDistanceThreshold this is one of the condition to trigger the slipping
    *           detection: the amount of slipping has to be greater than this threshold.
    * @param filterBreakFrequency this the break frequency to use for the internal low-pass filters.
    */
   public void configure(double forceMagnitudeThreshold, double velocityThreshold, double slippageDistanceThreshold,
                         double filterBreakFrequency)
   {
      this.forceMagnitudeThreshold.set(forceMagnitudeThreshold);
      this.velocityThreshold.set(velocityThreshold);
      this.slippageDistanceThreshold.set(slippageDistanceThreshold);
      alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency, dt));
   }

   private final FramePoint toeContactPointPosition = new FramePoint();

   public void initialize(FramePoint toeContactPointPosition)
   {
      this.toeContactPointPosition.setIncludingFrame(toeContactPointPosition);
      this.toeContactPointPosition.changeFrame(foot.getBodyFixedFrame());
      initialToePosition.setAndMatchFrame(toeContactPointPosition);
   }

   public void clear()
   {
      toeContactPointPosition.setToNaN(worldFrame);
      currentToePosition.setToNaN(worldFrame);
      toeForceFiltered.setToNaN();
      toeForceFiltered.reset();
      toeLinearVelocityFiltered.setToNaN();
      toeLinearVelocityFiltered.reset();
      isToeSlipping.set(false);
   }

   private final Wrench footWrench = new Wrench();
   private final FrameVector toeLinearVelocity = new FrameVector();
   private final FramePoint currentToePosition = new FramePoint();

   public void update()
   {
      footSwitch.computeAndPackFootWrench(footWrench);
      toeForceFiltered.update(footWrench.getLinearPartMagnitude());

      twistCalculator.getLinearVelocityOfBodyFixedPoint(foot, toeContactPointPosition, toeLinearVelocity);
      toeLinearVelocity.changeFrame(worldFrame);
      toeLinearVelocityFiltered.update(toeLinearVelocity);

      currentToePosition.setIncludingFrame(toeContactPointPosition);
      currentToePosition.changeFrame(worldFrame);
      double distance = initialToePosition.distance(currentToePosition);
      toeSlippageDistance.set(distance);

      if (toeForceFiltered.getDoubleValue() > forceMagnitudeThreshold.getDoubleValue())
         isToeSlipping.set(false);
      else if (toeLinearVelocityFiltered.length() < velocityThreshold.getDoubleValue())
         isToeSlipping.set(false);
      else if (toeSlippageDistance.getDoubleValue() < slippageDistanceThreshold.getDoubleValue())
         isToeSlipping.set(false);
      else
         isToeSlipping.set(true);
   }

   public boolean isToeSlipping()
   {
      return isToeSlipping.getBooleanValue();
   }
}
