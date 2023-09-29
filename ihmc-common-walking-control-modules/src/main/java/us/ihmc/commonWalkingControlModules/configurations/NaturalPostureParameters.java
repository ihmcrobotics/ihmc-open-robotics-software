package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public abstract class NaturalPostureParameters
{
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
   private final Vector3D pelvisPrivilegedOrientation = new Vector3D(0.0, 0.02, 0.0);
   private final Vector3D pelvisPrivilegedOrientationKp = new Vector3D(1500.0, 3000.0, 1000.0);
   private final Vector3D pelvisPrivilegedOrientationKd = new Vector3D(225.0, 450.0, 150.0);
   private final Vector3D pelvisWeights = new Vector3D(1.0, 1.0, 1.0);

   private final FrameYawPitchRoll spineNaturalPostureOrientationKp = new FrameYawPitchRoll(ReferenceFrame.getWorldFrame(), 0.0, 100.0, 100.0);

   private final ArrayList<OneDofJointPrivilegedParameters> parametersList = new ArrayList<>();
   /* ~~~~~~~~~~ End NaturalPosturePrivilegedConfigurationControllerParameters ~~~~~~~~~~ */

   public NaturalPostureParameters(FullHumanoidRobotModel fullRobotModel)
   {
      if (getUseSpinePrivilegedCommand())
      {
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW), 0.0, 300.0, 45.0, 1.0));
         /* These were not used in the original class, but they had gains assigned. Thus, we leave them here for posterity. */
         //      jointPrivilegedParametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH), 0.0, 50.0, 7.5, 1.0));
         //      jointPrivilegedParametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL), 0.0, 50.0, 7.5, 1.0));
      }

      for (RobotSide side : RobotSide.values)
      {
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_YAW), 0.0, 80.0, 12.0, 1.0));
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_PITCH), 0.1, 80.0, 12.0, 1.0));
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_ROLL), 0.0, 80.0, 12.0, 1.0));

         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.ELBOW_PITCH), -0.2, 30.0, 4.5, 1.0));

         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.WRIST_YAW), 0.0, 40.0, 6.0, 1.0));
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.FIRST_WRIST_PITCH), 0.0, 40.0, 6.0, 1.0));
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getArmJoint(side, ArmJointName.WRIST_ROLL), 0.0, 40.0, 6.0, 1.0));

         //TODO hip and knee values were not used, was this intended?

         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getLegJoint(side, LegJointName.ANKLE_PITCH), 0.0, 4.0, 0.6, 1.0));
         parametersList.add(new OneDofJointPrivilegedParameters(fullRobotModel.getLegJoint(side, LegJointName.ANKLE_ROLL), 0.0, 4.0, 0.6, 1.0));
      }
   }

   public ArrayList<OneDofJointPrivilegedParameters> getJointPrivilegedParametersList()
   {
      return parametersList;
   }

   public abstract String[] getJointsWithRestrictiveLimits();

   public abstract JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName);

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

   public static class OneDofJointPrivilegedParameters
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

   public BodyPrivilegedParameters getPelvisPrivilegedParameters()
   {
      return new BodyPrivilegedParameters(pelvisPrivilegedOrientation, pelvisPrivilegedOrientationKp, pelvisPrivilegedOrientationKd, pelvisWeights);
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
