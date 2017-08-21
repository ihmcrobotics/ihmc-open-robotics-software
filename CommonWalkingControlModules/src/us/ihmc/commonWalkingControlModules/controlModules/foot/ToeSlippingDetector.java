package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.ToeSlippingDetectorParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ToeSlippingDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoDouble alpha;
   private final AlphaFilteredYoVariable toeForceFiltered;
   private final AlphaFilteredYoFrameVector toeLinearVelocityFiltered;

   private final YoFramePoint initialToePosition;
   private final YoDouble toeSlippageDistance;

   private final YoDouble forceMagnitudeThreshold;
   private final YoDouble velocityThreshold;
   private final YoDouble slippageDistanceThreshold;

   private final YoBoolean isToeSlipping;

   private final double dt;
   private final RigidBody foot;
   private final FootSwitchInterface footSwitch;

   public ToeSlippingDetector(String namePrefix, double controlDT, RigidBody foot, FootSwitchInterface footSwitch,
                              YoVariableRegistry parentRegistry)
   {
      dt = controlDT;
      this.foot = foot;
      this.footSwitch = footSwitch;
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      alpha = new YoDouble(namePrefix + "AlphaFilter", registry);
      toeForceFiltered = new AlphaFilteredYoVariable(namePrefix + "ToeForceFiltered", "", registry, alpha);
      toeLinearVelocityFiltered = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "ToeLinearVelocityFiltered", "", registry, alpha,
                                                                                              worldFrame);

      initialToePosition = new YoFramePoint(namePrefix + "ToeInitial", worldFrame, registry);
      toeSlippageDistance = new YoDouble(namePrefix + "ToeSlippageDistance", registry);

      forceMagnitudeThreshold = new YoDouble(namePrefix + "ForceMagnitudeThreshold", registry);
      velocityThreshold = new YoDouble(namePrefix + "VelocityThreshold", registry);
      slippageDistanceThreshold = new YoDouble(namePrefix + "SlippageDistanceThreshold", registry);

      isToeSlipping = new YoBoolean(namePrefix + "IsToeSlipping", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Configure the parameters for this module.
    * <p>
    * The {@code ToeSlippingDetector} is meant to detect if the toe of the support performing
    * toe-off is strongly slipping. When this is the case, the walking controller will attempt to
    * stop the toe-off by swinging the foot.
    * </p>
    * @param parameters contain the values that the internal parameters will be set to.
    * @see ToeSlippingDetectorParameters
    */
   public void configure(ToeSlippingDetectorParameters parameters)
   {
      this.forceMagnitudeThreshold.set(parameters.getForceMagnitudeThreshold());
      this.velocityThreshold.set(parameters.getVelocityThreshold());
      this.slippageDistanceThreshold.set(parameters.getSlippageDistanceThreshold());
      alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(parameters.getFilterBreakFrequency(), dt));
   }

   private final FramePoint3D toeContactPointPosition = new FramePoint3D();

   public void initialize(FramePoint3D toeContactPointPosition)
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
   private final Twist footTwist = new Twist();
   private final FrameVector3D toeLinearVelocity = new FrameVector3D();
   private final FramePoint3D currentToePosition = new FramePoint3D();

   public void update()
   {
      footSwitch.computeAndPackFootWrench(footWrench);
      toeForceFiltered.update(footWrench.getLinearPartMagnitude());

      foot.getBodyFixedFrame().getTwistOfFrame(footTwist);
      footTwist.getLinearVelocityOfPointFixedInBodyFrame(toeLinearVelocity, toeContactPointPosition);
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
