package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.tools.UnitConversions;

public class KinematicsStreamingToolboxParameters
{
   private double centerOfMassSafeMargin;
   private double centerOfMassHoldWeight;
   private double publishingSolutionPeriod;

   private double defaultArmMessageWeight;
   private double defaultNeckMessageWeight;
   private final Vector3D defaultPelvisMessageLinearWeight = new Vector3D();
   private final Vector3D defaultPelvisMessageAngularWeight = new Vector3D();
   private final Vector3D defaultChestMessageAngularWeight = new Vector3D();

   private double defaultPelvisMessageLockWeight;
   private double defaultChestMessageLockWeight;

   private double defaultLinearWeight;
   private double defaultAngularWeight;

   private double defaultLinearRateLimit;
   private double defaultAngularRateLimit;
   private double outputJointVelocityScale;

   private boolean minimizeAngularMomentum;
   private boolean minimizeLinearMomentum;
   private double angularMomentumWeight;
   private double linearMomentumWeight;

   private double defaultStreamingBlendingDuration;

   private double inputPoseLPFBreakFrequency;
   private double inputWeightDecayDuration;
   private double inputVelocityDecayDuration;
   private double inputAccelerationDecayDuration;
   private boolean useStreamingPublisher;
   private double publishingPeriod;

   private final KinematicsStreamingToolboxConfigurationMessage defaultConfiguration = new KinematicsStreamingToolboxConfigurationMessage();

   public static KinematicsStreamingToolboxParameters defaultParameters()
   {
      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      return parameters;
   }

   public void setDefault()
   {
      centerOfMassSafeMargin = 0.05;
      centerOfMassHoldWeight = 0.001;
      publishingSolutionPeriod = UnitConversions.hertzToSeconds(60.0);
      defaultArmMessageWeight = 10.0;
      defaultNeckMessageWeight = 10.0;
      defaultPelvisMessageLinearWeight.set(2.5, 2.5, 2.5);
      defaultPelvisMessageAngularWeight.set(1.0, 1.0, 1.0);
      defaultChestMessageAngularWeight.set(0.75, 0.75, 0.75);

      defaultPelvisMessageLockWeight = 1000.0;
      defaultChestMessageLockWeight = 1000.0;

      defaultLinearWeight = 20.0;
      defaultAngularWeight = 1.0;

      defaultLinearRateLimit = 1.5;
      defaultAngularRateLimit = 10.0;
      outputJointVelocityScale = 0.75;

      minimizeAngularMomentum = true;
      minimizeLinearMomentum = false;
      angularMomentumWeight = 0.125;
      linearMomentumWeight = 0.0;

      defaultStreamingBlendingDuration = 2.0;

      inputPoseLPFBreakFrequency = 4.0;
      inputWeightDecayDuration = 3.0;
      inputVelocityDecayDuration = 0.5;
      inputAccelerationDecayDuration = 0.1;

      useStreamingPublisher = true;
      publishingPeriod = 5.0 * 0.006;

      defaultConfiguration.setLockPelvis(false);
      defaultConfiguration.setLockChest(false);
      defaultConfiguration.setEnableLeftArmJointspace(true);
      defaultConfiguration.setEnableRightArmJointspace(true);
      defaultConfiguration.setEnableNeckJointspace(true);
      defaultConfiguration.setEnableLeftHandTaskspace(true);
      defaultConfiguration.setEnableRightHandTaskspace(true);
      defaultConfiguration.setEnableChestTaskspace(true);
      defaultConfiguration.setEnablePelvisTaskspace(true);
      defaultConfiguration.setLeftHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setRightHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setChestTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setPelvisTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
   }

   public double getCenterOfMassSafeMargin()
   {
      return centerOfMassSafeMargin;
   }

   public double getCenterOfMassHoldWeight()
   {
      return centerOfMassHoldWeight;
   }

   public double getPublishingSolutionPeriod()
   {
      return publishingSolutionPeriod;
   }

   public double getDefaultArmMessageWeight()
   {
      return defaultArmMessageWeight;
   }

   public double getDefaultNeckMessageWeight()
   {
      return defaultNeckMessageWeight;
   }

   public Vector3D getDefaultPelvisMessageLinearWeight()
   {
      return defaultPelvisMessageLinearWeight;
   }

   public Vector3D getDefaultPelvisMessageAngularWeight()
   {
      return defaultPelvisMessageAngularWeight;
   }

   public Vector3D getDefaultChestMessageAngularWeight()
   {
      return defaultChestMessageAngularWeight;
   }

   public double getDefaultPelvisMessageLockWeight()
   {
      return defaultPelvisMessageLockWeight;
   }

   public double getDefaultChestMessageLockWeight()
   {
      return defaultChestMessageLockWeight;
   }

   public double getDefaultLinearWeight()
   {
      return defaultLinearWeight;
   }

   public double getDefaultAngularWeight()
   {
      return defaultAngularWeight;
   }

   public double getDefaultLinearRateLimit()
   {
      return defaultLinearRateLimit;
   }

   public double getDefaultAngularRateLimit()
   {
      return defaultAngularRateLimit;
   }

   public double getOutputJointVelocityScale()
   {
      return outputJointVelocityScale;
   }

   public boolean isMinimizeAngularMomentum()
   {
      return minimizeAngularMomentum;
   }

   public boolean isMinimizeLinearMomentum()
   {
      return minimizeLinearMomentum;
   }

   public double getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   public double getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   public double getDefaultStreamingBlendingDuration()
   {
      return defaultStreamingBlendingDuration;
   }

   public double getInputPoseLPFBreakFrequency()
   {
      return inputPoseLPFBreakFrequency;
   }

   public double getInputWeightDecayDuration()
   {
      return inputWeightDecayDuration;
   }

   public double getInputVelocityDecayDuration()
   {
      return inputVelocityDecayDuration;
   }

   public double getInputAccelerationDecayDuration()
   {
      return inputAccelerationDecayDuration;
   }

   public boolean getUseStreamingPublisher()
   {
      return useStreamingPublisher;
   }

   public KinematicsStreamingToolboxConfigurationMessage getDefaultConfiguration()
   {
      return defaultConfiguration;
   }

   public void setCenterOfMassSafeMargin(double centerOfMassSafeMargin)
   {
      this.centerOfMassSafeMargin = centerOfMassSafeMargin;
   }

   public void setCenterOfMassHoldWeight(double centerOfMassHoldWeight)
   {
      this.centerOfMassHoldWeight = centerOfMassHoldWeight;
   }

   public void setPublishingSolutionPeriod(double publishingSolutionPeriod)
   {
      this.publishingSolutionPeriod = publishingSolutionPeriod;
   }

   public void setDefaultArmMessageWeight(double defaultArmMessageWeight)
   {
      this.defaultArmMessageWeight = defaultArmMessageWeight;
   }

   public void setDefaultNeckMessageWeight(double defaultNeckMessageWeight)
   {
      this.defaultNeckMessageWeight = defaultNeckMessageWeight;
   }

   public void setDefaultPelvisMessageLinearWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultPelvisMessageLinearWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultPelvisMessageAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultPelvisMessageAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultChestMessageAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultChestMessageAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultPelvisMessageLockWeight(double defaultPelvisMessageLockWeight)
   {
      this.defaultPelvisMessageLockWeight = defaultPelvisMessageLockWeight;
   }

   public void setDefaultChestMessageLockWeight(double defaultChestMessageLockWeight)
   {
      this.defaultChestMessageLockWeight = defaultChestMessageLockWeight;
   }

   public void setDefaultLinearWeight(double defaultLinearWeight)
   {
      this.defaultLinearWeight = defaultLinearWeight;
   }

   public void setDefaultAngularWeight(double defaultAngularWeight)
   {
      this.defaultAngularWeight = defaultAngularWeight;
   }

   public void setDefaultLinearRateLimit(double defaultLinearRateLimit)
   {
      this.defaultLinearRateLimit = defaultLinearRateLimit;
   }

   public void setDefaultAngularRateLimit(double defaultAngularRateLimit)
   {
      this.defaultAngularRateLimit = defaultAngularRateLimit;
   }

   public void setOutputJointVelocityScale(double outputJointVelocityScale)
   {
      this.outputJointVelocityScale = outputJointVelocityScale;
   }

   public void setMinimizeAngularMomentum(boolean minimizeAngularMomentum)
   {
      this.minimizeAngularMomentum = minimizeAngularMomentum;
   }

   public void setMinimizeLinearMomentum(boolean minimizeLinearMomentum)
   {
      this.minimizeLinearMomentum = minimizeLinearMomentum;
   }

   public void setAngularMomentumWeight(double angularMomentumWeight)
   {
      this.angularMomentumWeight = angularMomentumWeight;
   }

   public void setLinearMomentumWeight(double linearMomentumWeight)
   {
      this.linearMomentumWeight = linearMomentumWeight;
   }

   public void setDefaultStreamingBlendingDuration(double defaultStreamingBlendingDuration)
   {
      this.defaultStreamingBlendingDuration = defaultStreamingBlendingDuration;
   }

   public void setInputPoseLPFBreakFrequency(double inputPoseLPFBreakFrequency)
   {
      this.inputPoseLPFBreakFrequency = inputPoseLPFBreakFrequency;
   }

   public void setInputWeightDecayDuration(double inputWeightDecayDuration)
   {
      this.inputWeightDecayDuration = inputWeightDecayDuration;
   }

   public void setInputVelocityDecayDuration(double inputVelocityDecayDuration)
   {
      this.inputVelocityDecayDuration = inputVelocityDecayDuration;
   }

   public void setInputAccelerationDecayDuration(double inputAccelerationDecayDuration)
   {
      this.inputAccelerationDecayDuration = inputAccelerationDecayDuration;
   }

   public void setUseStreamingPublisher(boolean useStreamingPublisher)
   {
      this.useStreamingPublisher = useStreamingPublisher;
   }

   public void setPublishingPeriod(double publishingPeriod)
   {
      this.publishingPeriod = publishingPeriod;
   }

   public double getPublishingPeriod()
   {
      return publishingPeriod;
   }
}
