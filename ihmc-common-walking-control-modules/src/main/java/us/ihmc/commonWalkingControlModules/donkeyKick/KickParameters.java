package us.ihmc.commonWalkingControlModules.donkeyKick;

public class KickParameters
{
   private static final double desiredCoMHeight = 0.85;

   private static final double coefficientOfFriction = 0.75;

   // When computing the touchdown position to remain stable, this is the extra length to step, as a safety margin
   private static final double extraStepLengthInTouchdownForSafety = 0.25; // m
   // When computing the touchdown positino to remain stable, this is the minimum step length
   private static final double minTouchdownStepDistance = 0.15; // m

   // This is how long we can estimate the force will be applied. This is used to determine how hard the foot should push for
   private static final double estimatedImpactDuration = 0.05; // s
   // This is the desired height of the foot at the end of chambering, relative to stance
   private static final double kickChamberHeight = 0.18; // m
   // This is the position of the foot at the end of chambering, relative to stance.
   private static final double kickChamberX = 0.25; // m
   // This is the mass of the kick leg to assume in order to generate the right amount of angular momentum
   private static final double kickLegMass = 100.0; // kg
   private static final double kickPickUpVelocity = 0.25; // m/s

   // This is a shift of the desired CoP position towards the inside of the foot when computing the dynamic planner.
   private static final double copShiftInside = 0.02; // m
   // This cheats the goal position of hte ACP at the touchdown event towards the inside, trying to incentivize the ACP position to be further
   // horizontally the stance foot
   private static final double endACPCheatInside = 0.02; // m

   // Defines how far past the target the touchdown position should be. Positive is further away, negative is closer.
   private static final double desiredTouchdownPositionRelativeToTarget = -0.15; // m
   private static final double touchdownHeightSpeed = 1.0; // m /s

   private static final double preShiftWeightDistribution = 0.75; // percentage
   private static final double preShiftDuration = 2.5; // s
   private static final double shiftDuration = 0.5; // s // TODO make this a computed value
   private static final double chamberDuration = 0.25; // s
   private static final double pushDuration = 0.2; // s

   private static final double maxAnkleVelocity = 2.5; // rad/s

   private static final double kickShiftProximityThreshold = 0.01;
   private static final double kickMinTimeInContact = 0.05;
   private static final double minFractionThroughTouchdownToDetectContact = 0.85;

   private static final double kickPenetrationThreshold = 0.04;

   private static final double kpICP = 15.0;
   private static final double maxCMPFeedack = 0.10;
   private static final double minCMPFeedack = 0.10;
   private static final double cmpDistanceInside = 0.005;
   private static final double cmpDistanceInsideInTouchdown = 0.025;
   private static final double maxCMPRate = 5.0;

   private final KickingWBCCParameters fastWalkingWBCCParameters = new KickingWBCCParameters();

   public double getDesiredCoMHeight()
   {
      return desiredCoMHeight;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public double getExtraStepLengthInTouchdownForSafety()
   {
      return extraStepLengthInTouchdownForSafety;
   }

   public double getMinTouchdownStepDistance()
   {
      return minTouchdownStepDistance;
   }

   public double getEstimatedImpactDuration()
   {
      return estimatedImpactDuration;
   }

   public double getKickChamberX()
   {
      return kickChamberX;
   }

   public double getKickChamberHeight()
   {
      return kickChamberHeight;
   }

   public double getKickLegMass()
   {
      return kickLegMass;
   }

   public double getKickPickUpVelocity()
   {
      return kickPickUpVelocity;
   }

   public double getCopShiftInside()
   {
      return copShiftInside;
   }

   public double getEndACPCheatInside()
   {
      return endACPCheatInside;
   }

   public double getDesiredTouchdownPositionRelativeToTarget()
   {
      return desiredTouchdownPositionRelativeToTarget;
   }

   public double getTouchdownHeightSpeed()
   {
      return touchdownHeightSpeed;
   }

   public double getPreShiftWeightDistribution()
   {
      return preShiftWeightDistribution;
   }

   public double getPreShiftDuration()
   {
      return preShiftDuration;
   }

   public double getShiftDuration()
   {
      return shiftDuration;
   }

   public double getChamberDuration()
   {
      return chamberDuration;
   }

   public double getPushDuration()
   {
      return pushDuration;
   }

   public double getMaxAnkleVelocity()
   {
      return maxAnkleVelocity;
   }

   public double getKickShiftProximityThreshold()
   {
      return kickShiftProximityThreshold;
   }

   public double getKickMinTimeInContact()
   {
      return kickMinTimeInContact;
   }

   public double getMinFractionThroughTouchdownToDetectContact()
   {
      return minFractionThroughTouchdownToDetectContact;
   }

   public double getKpICP()
   {
      return kpICP;
   }

   public double getMaxCMPFeedack()
   {
      return maxCMPFeedack;
   }

   public double getMinCMPFeedack()
   {
      return minCMPFeedack;
   }

   public double getCmpDistanceInside()
   {
      return cmpDistanceInside;
   }

   public double getCmpDistanceInsideInTouchdown()
   {
      return cmpDistanceInsideInTouchdown;
   }

   public double getMaxCMPRate()
   {
      return maxCMPRate;
   }

   public double getKickPenetrationThreshold()
   {
      return kickPenetrationThreshold;
   }

   public KickingWBCCParameters getKickingWBCCParameters()
   {
      return fastWalkingWBCCParameters;
   }
}
