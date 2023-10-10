package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.ArrayList;

public interface NaturalPostureParameters
{
   HumanoidRobotNaturalPosture getNaturalPosture(FullHumanoidRobotModel fullRobotModel);

   /* ~~~~~~~~~~ Begin NaturalPostureManager methods ~~~~~~~~~~ */
   String[] getJointsWithRestrictiveLimits();

   JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);
   /* ~~~~~~~~~~ End NaturalPostureManager methods ~~~~~~~~~~ */

   /* ~~~~~~~~~~ Begin NaturalPostureControllerParameters ~~~~~~~~~~ */
   boolean getDoNullSpaceProjectionForNaturalPosture();

   FrameYawPitchRoll getComAngleDesired();

   double getVelocityBreakFrequency();

   Vector3D getWeights();

   FrameYawPitchRoll getAngularComKpGains();

   FrameYawPitchRoll getAngularComKdGains();
   /* ~~~~~~~~~~ End NaturalPostureControllerParameters ~~~~~~~~~~ */

   /* ~~~~~~~~~~ Begin NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */
   ArrayList<OneDofJointPrivilegedParameters> getJointPrivilegedParametersList();

   BodyPrivilegedParameters getPelvisPrivilegedParameters();

   boolean getDoNullSpaceProjectionForPelvis();

   /* ~~~~~~~~~~ End NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */

   class OneDofJointPrivilegedParameters
   {
      private final String jointName;
      private final double privilegedOrientation;
      private final double kp;
      private final double kd;
      private final double weight;
      private final boolean isPrimaryTask;

      public OneDofJointPrivilegedParameters(String jointName, double privilegedOrientation, double kp, double kd, double weight)
      {
         this(jointName, privilegedOrientation, kp, kd, weight, false);
      }

      public OneDofJointPrivilegedParameters(String jointName, double privilegedOrientation, double kp, double kd, double weight, boolean isPrimaryTask)
      {
         this.jointName = jointName;
         this.privilegedOrientation = privilegedOrientation;
         this.kp = kp;
         this.kd = kd;
         this.weight = weight;
         this.isPrimaryTask = isPrimaryTask;
      }

      public String getJointName()
      {
         return jointName;
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

      /**
       * This flag is TRUE when the privileged task should be considered primary and be added with the rest of the tasks in the QP. This flag is FALSE when the
       * privileged task is secondary and thus should be projected into the nullspace of the primary tasks (this is the usual case for privileged tasks).
       */
      public boolean isPrimaryTask()
      {
         return isPrimaryTask;
      }
   }

   class BodyPrivilegedParameters
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
