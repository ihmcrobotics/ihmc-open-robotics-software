package us.ihmc.commonWalkingControlModules.donkeyKick;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.*;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;

public class KickingWBCCParameters
{
   private final PID3DGains chestOrientationGains = new DefaultPID3DGains();
   private final static double chestOrientationWeight = 50.0;

   private final PID3DGains pelvisOrientationGains = new DefaultPID3DGains();
   private final static double pelvisOrientationWeight = 50.0;

   private final static double stanceFootWeight = SolverWeightLevels.FOOT_SUPPORT_WEIGHT;

   private final PID3DGains chamberFootGains = new DefaultPID3DGains();
   private final static double chamberFootWeight = 50.0;

   private final PID3DGains touchdownFootGains = new DefaultPID3DGains();
   private final static double touchdownFootWeight = 50.0;

   private final PID3DGains settlingFootGains = new DefaultPID3DGains();

   private final PID3DGains pushFootGains = new DefaultPID3DGains();
   private final static double pushFootWeight = 100.0;

   private final PDGains kickingAnkleGains = new PDGains();
   private final PDGains kickingHipYawGains = new PDGains();
   private final static double kickingAnkleWeight = 50.0;
   private final static double kickingHipYawWeight = 50.0;

   private final static double angularMomentumRateWeight = 5.0;
   private final static Vector3DReadOnly linearMomentumRateWeight = new Vector3D(0.1, 0.1, 0.1);

   private final PDGains standingHeightControlGains = new PDGains();

   private final static double desiredCoPPositionWeight = 10.0;

   public KickingWBCCParameters()
   {
      standingHeightControlGains.setKp(50.0);
      standingHeightControlGains.setZeta(0.7);

      chamberFootGains.setProportionalGains(300.0, 300.0, 100.0);
      chamberFootGains.setDerivativeGains(GainCalculator.computeDerivativeGain(chamberFootGains.getProportionalGains()[0], 0.7),
                                          GainCalculator.computeDerivativeGain(chamberFootGains.getProportionalGains()[1], 0.7),
                                          GainCalculator.computeDerivativeGain(chamberFootGains.getProportionalGains()[2], 0.2));

      touchdownFootGains.setProportionalGains(300.0);
      touchdownFootGains.setDerivativeGains(GainCalculator.computeDerivativeGain(300.0, 0.7));

      settlingFootGains.setProportionalGains(200.0);
      settlingFootGains.setDerivativeGains(GainCalculator.computeDerivativeGain(200.0, 0.7));

      pushFootGains.setProportionalGains(100.0, 50.0, 5.0);
      pushFootGains.setDerivativeGains(GainCalculator.computeDerivativeGain(pushFootGains.getProportionalGains()[0], 0.7),
                                       GainCalculator.computeDerivativeGain(pushFootGains.getProportionalGains()[1], 0.7),
                                       GainCalculator.computeDerivativeGain(pushFootGains.getProportionalGains()[2], 0.7));

      chestOrientationGains.setProportionalGains(40.0);
      chestOrientationGains.setDerivativeGains(GainCalculator.computeDerivativeGain(40.0, 0.8));

      pelvisOrientationGains.setProportionalGains(100.0);
      pelvisOrientationGains.setDerivativeGains(GainCalculator.computeDerivativeGain(100.0, 0.8));

      kickingAnkleGains.setKp(50.0);
      kickingAnkleGains.setZeta(0.75);
      kickingHipYawGains.setKp(50.0);
      kickingHipYawGains.setZeta(0.75);
   }

   public PDGainsReadOnly getStandingHeightControlGains()
   {
      return standingHeightControlGains;
   }

   public double getChestOrientationWeight()
   {
      return chestOrientationWeight;
   }

   public double getPelvisOrientationWeight()
   {
      return pelvisOrientationWeight;
   }

   public double getChamberFootWeight()
   {
      return chamberFootWeight;
   }

   public double getTouchdownFootWeight()
   {
      return touchdownFootWeight;
   }

   public PID3DGainsReadOnly getChamberFootGains()
   {
      return chamberFootGains;
   }

   public PID3DGainsReadOnly getTouchdownFootGains()
   {
      return touchdownFootGains;
   }

   public PID3DGainsReadOnly getSettlingFootGains()
   {
      return settlingFootGains;
   }

   public double getPushFootWeight()
   {
      return pushFootWeight;
   }

   public PID3DGainsReadOnly getPushFootGains()
   {
      return pushFootGains;
   }

   public PID3DGainsReadOnly getChestOrientationGains()
   {
      return chestOrientationGains;
   }

   public PID3DGainsReadOnly getPelvisOrientationGains()
   {
      return pelvisOrientationGains;
   }

   public Vector3DReadOnly getLinearMomentumRateWeight()
   {
      return linearMomentumRateWeight;
   }

   public double getAngularMomentumRateWeight()
   {
      return angularMomentumRateWeight;
   }

   public double getStanceFootWeight()
   {
      return stanceFootWeight;
   }

   public PDGainsReadOnly getKickingAnkleGains()
   {
      return kickingAnkleGains;
   }

   public PDGainsReadOnly getKickingHipYawGains()
   {
      return kickingHipYawGains;
   }

   public double getKickingAnkleWeight()
   {
      return kickingAnkleWeight;
   }

   public double getKickingHipYawWeight()
   {
      return kickingHipYawWeight;
   }

   public double getDesiredCoPPositionWeight()
   {
      return desiredCoPPositionWeight;
   }
}
