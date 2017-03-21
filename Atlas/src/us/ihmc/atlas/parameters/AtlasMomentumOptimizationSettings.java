package us.ihmc.atlas.parameters;

import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // defaults for unscaled model:
   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 4.0;
   private static final double defaultRhoRateDefaultWeight = 0.002;
   private static final double defaultRhoRateHighWeight = 0.05;

   private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
   private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(0.5, 0.5, 0.05);
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

   private final Vector3D defaultAngularFootWeight = new Vector3D(0.5, 0.5, 0.5);
   private final Vector3D defaultLinearFootWeight = new Vector3D(30.0, 30.0, 30.0);
   private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D pelvisAngularWeight = new Vector3D(5.0, 5.0, 5.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1;
   private final Vector2D copWeight = new Vector2D(100.0, 200.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
   private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);

   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;

   private final double neckJointspaceWeight = 1.0;
   private final double spineJointspaceWeightYaw = 15.0;
   private final double spineJointspaceWeightPitch = 45.0;
   private final double spineJointspaceWeightRoll = 45.0;
   private final double armJointspaceWeight = 1.0;
   private final TObjectDoubleHashMap<String> jointspaceWeights = new TObjectDoubleHashMap<>();

   private final double neckUserModeWeight = 1.0;
   private final double spineUserModeWeight = 200.0;
   private final double armUserModeWeight = 50.0;
   private final TObjectDoubleHashMap<String> userModeWeights = new TObjectDoubleHashMap<>();

   private final Vector3D headAngularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final Vector3D handAngularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Map<String, Vector3D> taskspaceAngularWeights = new HashMap<>();

   private final Vector3D handLinearWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Map<String, Vector3D> taskspaceLinearWeights = new HashMap<>();

   public AtlasMomentumOptimizationSettings(AtlasJointMap jointMap)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      rhoWeight = defaultRhoWeight / scale;
      rhoMin = defaultRhoMin * scale;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      linearMomentumWeight.scale(1.0 / scale);
      highLinearMomentumWeightForRecovery.scale(1.0 / scale);
      angularMomentumWeight.scale(1.0 / scale);

      for (SpineJointName jointName : jointMap.getSpineJointNames())
         userModeWeights.put(jointMap.getSpineJointName(jointName), spineUserModeWeight);

      jointspaceWeights.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), spineJointspaceWeightYaw);
      jointspaceWeights.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), spineJointspaceWeightPitch);
      jointspaceWeights.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), spineJointspaceWeightRoll);

      for (ArmJointName jointName : jointMap.getArmJointNames())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            jointspaceWeights.put(jointMap.getArmJointName(robotSide, jointName), armJointspaceWeight);
            userModeWeights.put(jointMap.getArmJointName(robotSide, jointName), armUserModeWeight);
         }
      }

      for (NeckJointName jointName : jointMap.getNeckJointNames())
      {
         jointspaceWeights.put(jointMap.getNeckJointName(jointName), neckJointspaceWeight);
         userModeWeights.put(jointMap.getNeckJointName(jointName), neckUserModeWeight);
      }

      taskspaceAngularWeights.put(jointMap.getChestName(), chestAngularWeight);
      taskspaceAngularWeights.put(jointMap.getHeadName(), headAngularWeight);
      for (RobotSide robotSide : RobotSide.values)
      {
         taskspaceAngularWeights.put(jointMap.getHandName(robotSide), handAngularWeight);
         taskspaceLinearWeights.put(jointMap.getHandName(robotSide), handLinearWeight);
      }

      nContactableBodies = jointMap.getNumberOfContactableBodies();
   }

   /** @inheritDoc */
   @Override
   public Vector3D getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighLinearMomentumWeightForRecovery()
   {
      return highLinearMomentumWeightForRecovery;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   /** @inheritDoc */
   @Override
   public double getJointAccelerationWeight()
   {
      return jointAccelerationWeight;
   }

   /** @inheritDoc */
   @Override
   public double getJointJerkWeight()
   {
      return jointJerkWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoWeight()
   {
      return rhoWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoMin()
   {
      return rhoMin;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateDefaultWeight()
   {
      return rhoRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public double getRhoRateHighWeight()
   {
      return rhoRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPWeight()
   {
      return copWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return copRateDefaultWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector2D getCoPRateHighWeight()
   {
      return copRateHighWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadUserModeWeight()
   {
      return neckUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadJointspaceWeight()
   {
      return neckJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHeadAngularWeight()
   {
      return headAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getChestAngularWeight()
   {
      return chestAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getPelvisAngularWeight()
   {
      return pelvisAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getDefaultLinearFootWeight()
   {
      return defaultLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getDefaultAngularFootWeight()
   {
      return defaultAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighLinearFootWeight()
   {
      return highLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHighAngularFootWeight()
   {
      return highAngularFootWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandUserModeWeight()
   {
      return armUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getChestUserModeWeight()
   {
      return spineUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandJointspaceWeight()
   {
      return armJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandAngularTaskspaceWeight()
   {
      return handAngularWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandLinearTaskspaceWeight()
   {
      return handLinearWeight;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return nContactPointsPerContactableBody;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }

   /** @inheritDoc */
   @Override
   public int getRhoSize()
   {
      return  nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public TObjectDoubleHashMap<String> getJointspaceWeights()
   {
      return jointspaceWeights;
   }

   /** @inheritDoc */
   @Override
   public TObjectDoubleHashMap<String> getUserModeWeights()
   {
      return userModeWeights;
   }

   /** @inheritDoc */
   @Override
   public Map<String, Vector3D> getTaskspaceAngularWeights()
   {
      return taskspaceAngularWeights;
   }

   /** @inheritDoc */
   @Override
   public Map<String, Vector3D> getTaskspaceLinearWeights()
   {
      return taskspaceLinearWeights;
   }
}
