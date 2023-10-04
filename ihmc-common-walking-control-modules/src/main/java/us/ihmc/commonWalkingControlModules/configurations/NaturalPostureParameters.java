package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

import java.util.ArrayList;

public interface NaturalPostureParameters
{
   /* ~~~~~~~~~~ Begin NaturalPostureManager methods ~~~~~~~~~~ */
   String[] getJointsWithRestrictiveLimits();

   JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);
   /* ~~~~~~~~~~ End NaturalPostureManager methods ~~~~~~~~~~ */

   /* ~~~~~~~~~~ Begin NaturalPostureControllerParameters ~~~~~~~~~~ */
   boolean getdoNullSpaceProjectionForNaturalPosture();

   FrameYawPitchRoll getComAngleDesired();

   double getVelocityBreakFrequency();

   Vector3D getQPWeights();

   FrameYawPitchRoll getAngularComKpGains();

   FrameYawPitchRoll getAngularComKdGains();
   /* ~~~~~~~~~~ End NaturalPostureControllerParameters ~~~~~~~~~~ */

   /* ~~~~~~~~~~ Begin NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */
   ArrayList<OneDofJointPrivilegedParameters> getJointPrivilegedParametersList();

   BodyPrivilegedParameters getPelvisPrivilegedParameters();

   boolean getDoNullSpaceProjectionForPelvis();

   FrameYawPitchRoll getSpineNaturalPostureOrientationKp();

   double getSpineDamping();

   boolean getUseSpinePrivilegedCommand();

   boolean getUseSpineRollPitchJointCommands();
   /* ~~~~~~~~~~ End NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */

   class OneDofJointPrivilegedParameters
   {
      private final OneDoFJointBasics joint;
      private final double privilegedOrientation;
      private final double kp;
      private final double kd;
      private final double weight;

      public OneDofJointPrivilegedParameters(OneDoFJointBasics joint, double privilegedOrientation, double kp, double kd, double weight)
      {
         this.joint = joint;
         this.privilegedOrientation = privilegedOrientation;
         this.kp = kp;
         this.kd = kd;
         this.weight = weight;
      }

      public OneDoFJointBasics getJoint()
      {
         return joint;
      }

      public double getPrivilegedOrientation()
      {
         return privilegedOrientation;
      }

      public double getKp()
      {
         return kp;
      }

      public double getKd()
      {
         return kd;
      }

      public double getWeight()
      {
         return weight;
      }
   }

   public static class BodyPrivilegedParameters
   {
      private final Vector3D privilegedOrientation;
      private final Vector3D kp;
      private final Vector3D kd;
      private final Vector3D weight;

      public BodyPrivilegedParameters(Vector3D privilegedOrientation, Vector3D kp, Vector3D kd, Vector3D weight)
      {
         this.privilegedOrientation = privilegedOrientation;
         this.kp = kp;
         this.kd = kd;
         this.weight = weight;
      }

      public Vector3D getPrivilegedOrientation()
      {
         return privilegedOrientation;
      }

      public Vector3D getKpGain()
      {
         return kp;
      }

      public Vector3D getKdGain()
      {
         return kd;
      }

      public Vector3D getQPWeight()
      {
         return weight;
      }
   }
}
