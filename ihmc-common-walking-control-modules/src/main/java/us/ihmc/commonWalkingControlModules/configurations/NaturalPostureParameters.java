package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class NaturalPostureParameters
{
   public abstract String[] getJointsWithRestrictiveLimits();

   public abstract JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);

   /* ~~~~~~~~~~ Begin NaturalPostureControllerParameters ~~~~~~~~~~ */
   private final Vector3D npQPWeights = new Vector3D(0.01, 0.01, 1);
   private final FrameYawPitchRoll comAngleDesired = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll angularComKpGains = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
   private final FrameYawPitchRoll angularComKdGains = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 5.0, 5.0, 5.0);

   public boolean getdoNullSpaceProjectionForNaturalPosture()
   {
      return true;
   }

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
   /* ~~~~~~~~~~ End NaturalPostureControllerParameters ~~~~~~~~~~ */

   /* ~~~~~~~~~~ Begin NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */
   private final FrameYawPitchRoll pelvisPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.02, 0.0);
   private final FrameYawPitchRoll pelvisPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1000.0, 3000.0, 1500.0);
   private final FrameYawPitchRoll pelvisPrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 150.0, 450.0, 225.0);
   private final FrameYawPitchRoll pelvisQPWeights = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

   private final FrameYawPitchRoll spineNaturalPostureOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 100.0, 100.0);

   private final FrameYawPitchRoll spinePrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll spinePrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 300.0, 50.0, 50.0);
   private final FrameYawPitchRoll spinePrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 45.0, 7.5, 7.5);
   private final FrameYawPitchRoll spinePrivilegedOrientationQPWeights = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0);

   private final FrameYawPitchRoll shoulderPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.1, 0.0);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 80.0, 80.0, 80.0);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 12.0, 12.0, 12.0);
   private final FrameYawPitchRoll shoulderPrivilegedOrientationQPWeight = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

   // Pitch only
   private final FrameYawPitchRoll elbowPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, -0.2, 0.0);
   private final FrameYawPitchRoll elbowPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 30.0, 0.0);
   private final FrameYawPitchRoll elbowPrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 4.5, 0.0);
   private final FrameYawPitchRoll elbowPrivilegedQPWeight = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 1.0, 0.0);

   private final FrameYawPitchRoll wristPrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll wristPrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 40.0, 40.0, 40.0);
   private final FrameYawPitchRoll wristPrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 6.0, 6.0, 6.0);
   private final FrameYawPitchRoll wristPrivilegedQPWeight = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 1.0, 1.0, 1.0);

   // Pitch and Roll
   private final FrameYawPitchRoll anklePrivilegedOrientation = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   private final FrameYawPitchRoll anklePrivilegedOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 4.0, 4.0);
   private final FrameYawPitchRoll anklePrivilegedOrientationKd = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 0.6, 0.6);
   private final FrameYawPitchRoll anklePrivilegedQPWeight = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 1.0, 1.0);


   public boolean getDoNullSpaceProjectionForPelvis()
   {
      return true;
   }

   public FrameYawPitchRoll getSpineNaturalPostureOrientationKp()
   {
      return spineNaturalPostureOrientationKp;
   }

   public double getSpineDamping()
   {
      return 0.7;
   }

   public boolean getUseSpinePrivilegedCommand()
   {
      return true;
   }

   public boolean getUseSpineRollPitchJointCommands()
   {
      return true;
   }

   public JointPrivilegedParameters getPelvisPrivilegedParameters()
   {
      return new JointPrivilegedParameters(pelvisPrivilegedOrientation, pelvisPrivilegedOrientationKp, pelvisPrivilegedOrientationKd, pelvisQPWeights);
   }

   public JointPrivilegedParameters getSpinePrivilegedParameters()
   {
      return new JointPrivilegedParameters(spinePrivilegedOrientation,
                                           spinePrivilegedOrientationKp,
                                           spinePrivilegedOrientationKd,
                                           spinePrivilegedOrientationQPWeights);
   }

   public JointPrivilegedParameters getShoulderPrivilegedParameters()
   {
      return new JointPrivilegedParameters(shoulderPrivilegedOrientation,
                                           shoulderPrivilegedOrientationKp,
                                           shoulderPrivilegedOrientationKd,
                                           shoulderPrivilegedOrientationQPWeight);
   }

   public JointPrivilegedParameters getElbowPrivilegedParameters()
   {
      return new JointPrivilegedParameters(elbowPrivilegedOrientation,
                                           elbowPrivilegedOrientationKp,
                                           elbowPrivilegedOrientationKd,
                                           elbowPrivilegedQPWeight);
   }

   public JointPrivilegedParameters getWristPrivilegedParameters()
   {
      return new JointPrivilegedParameters(wristPrivilegedOrientation,
                                           wristPrivilegedOrientationKp,
                                           wristPrivilegedOrientationKd,
                                           wristPrivilegedQPWeight);
   }

   public JointPrivilegedParameters getAnklePrivilegedParameters()
   {
      return new JointPrivilegedParameters(anklePrivilegedOrientation,
                                           anklePrivilegedOrientationKp,
                                           anklePrivilegedOrientationKd,
                                           anklePrivilegedQPWeight);
   }

   public static class JointPrivilegedParameters
   {
      private final FrameYawPitchRoll privilegedOrientation;
      private final FrameYawPitchRoll kpGains;
      private final FrameYawPitchRoll kdGains;
      private final FrameYawPitchRoll qpWeights;

      public JointPrivilegedParameters(FrameYawPitchRoll privilegedOrientation,
                                       FrameYawPitchRoll kpGains,
                                       FrameYawPitchRoll kdGains,
                                       FrameYawPitchRoll qpWeights)
      {
         this.privilegedOrientation = privilegedOrientation;
         this.kpGains = kpGains;
         this.kdGains = kdGains;
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

      public FrameYawPitchRoll getKdGain()
      {
         return kdGains;
      }

      public FrameYawPitchRoll getQPWeight()
      {
         return qpWeights;
      }
   }
   /* ~~~~~~~~~~ End NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */
}
