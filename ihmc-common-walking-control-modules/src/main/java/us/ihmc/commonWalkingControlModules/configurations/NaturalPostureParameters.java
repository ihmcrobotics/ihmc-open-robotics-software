package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class NaturalPostureParameters
{
   public abstract String[] getJointsWithRestrictiveLimits();

   public abstract JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);

   /* Begin NaturalPostureControllerParameters */
   private final Vector3D npQPWeights = new Vector3D(0.01, 0.01, 1);
   private final FrameYawPitchRoll comAngleDesired = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll angularComKpGains = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
   private final FrameYawPitchRoll angularComKdGains = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 5.0, 5.0, 5.0);
   private final FrameYawPitchRoll pelvisPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.02, 0.0);
   private final FrameYawPitchRoll pelvisPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1000.0, 3000.0, 1500.0);
   private final FrameYawPitchRoll pelvisPrivilegedOrientationKdFactor = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.15, 0.15, 0.15);
   private final Vector3D pelvisQPWeights = new Vector3D(1, 1, 1);

   public FrameYawPitchRoll getComAngleDesired()
   {
      return comAngleDesired;
   }

   public double getVelocityBreakFrequency()
   {
      return 50.0;
   }

   public Vector3D getQPWeights()
   {
      return npQPWeights;
   }

   public FrameYawPitchRoll getAngularComKpGains()
   {
      return angularComKpGains;
   }

   public FrameYawPitchRoll getAngularComKdGains()
   {
      return angularComKdGains;
   }

   public FrameYawPitchRoll getPelvisPrivilegedOrientation()
   {
      return pelvisPrivilegedOrientation;
   }

   public FrameYawPitchRoll getPelvisPrivilegedOrientationKp()
   {
      return pelvisPrivilegedOrientationKp;
   }

   public FrameYawPitchRoll getPelvisPrivilegedOrientationKdFactor()
   {
      return pelvisPrivilegedOrientationKdFactor;
   }

   public Vector3D getPelvisQPWeights()
   {
      return pelvisQPWeights;
   }
   /* End NaturalPostureControllerParameters */

   /* Begin NaturalPosturePrivilegedConfigurationControllerParameters */
   private final FrameYawPitchRoll spineNaturalPostureOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 100.0, 100.0);

   private final FrameYawPitchRoll spinePrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll spinePrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 300.0, 50.0, 50.0);
   private final FrameYawPitchRoll spinePrivilegedOrientationKdFactor = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.15, 0.15, 0.15);
   private final FrameYawPitchRoll spinePrivilegedOrientationQPWeights = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0);

   private final FrameYawPitchRoll shoulderPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.1, 0.0);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 80.0, 80.0, 80.0);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationKdFactor = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.15, 0.15, 0.15);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationQPWeight = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

   private final double elbowPrivilegedOrientation = -0.2;
   private final double elbowPrivilegedOrientationKp = 30.0;
   private final double elbowPrivilegedOrientationKdFactor = 0.15;
   private final double elbowPrivilegedQPWeight = 1.0;

   private final double wristPrivilegedOrientation = 0.0;
   private final double wristPrivilegedOrientationKp = 40.0;
   private final double wristPrivilegedOrientationKdFactor = 6.0/40.0;
   private final double wristPrivilegedQPWeight = 1.0;

   private final double anklePrivilegedOrientation = 0.0;
   private final double anklePrivilegedOrientationKp = 4.0;
   private final double anklePrivilegedOrientationKdFactor = 0.6/4.0;
   private final double anklePrivilegedQPWeight = 1.0;


   public boolean getUseSpinePrivilegedCommand()
   {
      return true;
   }

   public boolean getUseSpineRollPitchJointCommands()
   {
      return true;
   }

   public MultiDOFJointPrivilegedParameters getSpinePrivilegedParameters()
   {
      return new MultiDOFJointPrivilegedParameters(spinePrivilegedOrientation,
                                                   spinePrivilegedOrientationKp,
                                                   spinePrivilegedOrientationKdFactor,
                                                   spinePrivilegedOrientationQPWeights);
   }

   public MultiDOFJointPrivilegedParameters getShoulderPrivilegedParameters()
   {
      return new MultiDOFJointPrivilegedParameters(shoulderPrivilegedOrientation,
                                                   shoulderPrivilegedOrientationKp,
                                                   shoulderPrivilegedOrientationKdFactor,
                                                   shoulderPrivilegedOrientationQPWeight);
   }

   public SingleDOFJointPrivilegedParameters getElbowPrivilegedParameters()
   {
      return new SingleDOFJointPrivilegedParameters(elbowPrivilegedOrientation,
                                                    elbowPrivilegedOrientationKp,
                                                    elbowPrivilegedOrientationKdFactor,
                                                    elbowPrivilegedQPWeight);
   }

   public SingleDOFJointPrivilegedParameters getWristPrivilegedParameters()
   {
      return new SingleDOFJointPrivilegedParameters(wristPrivilegedOrientation,
                                                    wristPrivilegedOrientationKp,
                                                    wristPrivilegedOrientationKdFactor,
                                                    wristPrivilegedQPWeight);
   }

   public SingleDOFJointPrivilegedParameters getAnklePrivilegedParameters()
   {
      return new SingleDOFJointPrivilegedParameters(anklePrivilegedOrientation,
                                                    anklePrivilegedOrientationKp,
                                                    anklePrivilegedOrientationKdFactor,
                                                    anklePrivilegedQPWeight);
   }

   public FrameYawPitchRoll getSpineNaturalPostureOrientationKp()
   {
      return spineNaturalPostureOrientationKp;
   }

   public double getSpineDamping()
   {
      return 0.7;
   }

   public class MultiDOFJointPrivilegedParameters
   {
      private final FrameYawPitchRoll privilegedOrientation;
      private final FrameYawPitchRoll kpGains;
      private final FrameYawPitchRoll kdGainFactors;
      private final FrameYawPitchRoll qpWeights;

      public MultiDOFJointPrivilegedParameters(FrameYawPitchRoll privilegedOrientation, FrameYawPitchRoll kpGains, FrameYawPitchRoll kdGainFactors, FrameYawPitchRoll qpWeights)
      {
         this.privilegedOrientation = privilegedOrientation;
         this.kpGains = kpGains;
         this.kdGainFactors = kdGainFactors;
         this.qpWeights = qpWeights;
      }

      public FrameYawPitchRoll getPrivilegedOrientation()
      {
         return privilegedOrientation;
      }

      public FrameYawPitchRoll getKpGain()
      {
         return kpGains;
      }

      public FrameYawPitchRoll getKdGainFactor()
      {
         return kdGainFactors;
      }

      public FrameYawPitchRoll getQPWeight()
      {
         return qpWeights;
      }
   }

   public class SingleDOFJointPrivilegedParameters
   {
      private final double privilegedOrientation;
      private final double kpGain;
      private final double kdGainFactor;
      private final double qpWeight;

      public SingleDOFJointPrivilegedParameters(double privilegedOrientation, double kpGain, double kdGainFactor, double qpWeight)
      {
         this.privilegedOrientation = privilegedOrientation;
         this.kpGain = kpGain;
         this.kdGainFactor = kdGainFactor;
         this.qpWeight = qpWeight;
      }

      public double getPrivilegedOrientation()
      {
         return privilegedOrientation;
      }

      public double getKpGain()
      {
         return kpGain;
      }

      public double getKdGainFactor()
      {
         return kdGainFactor;
      }

      public double getQPWeight()
      {
         return qpWeight;
      }
   }
}
