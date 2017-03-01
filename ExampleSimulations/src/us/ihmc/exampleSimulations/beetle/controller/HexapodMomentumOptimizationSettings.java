package us.ihmc.exampleSimulations.beetle.controller;

import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class HexapodMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
   private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(0.5, 0.5, 0.05);
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.0);

   private final Vector3D defaultAngularFootWeight = new Vector3D(0.5, 0.5, 0.5);
   private final Vector3D defaultLinearFootWeight = new Vector3D(30.0, 30.0, 30.0);
   private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final double chestUserModeWeight = 50.0;
   private final Vector3D pelvisAngularWeight = new Vector3D(5.0, 5.0, 5.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 1;
   private final int nContactableBodies = 6;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1;
   private final double rhoWeight = 0.00001;
   private final double rhoMin = 4.0;
   private final double rhoRateDefaultWeight = 0.002; // 0.005
   private final double rhoRateHighWeight = 0.05;
   private final Vector2D copWeight = new Vector2D(100.0, 200.0); //750.0, 1500.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0); //100000.0, 200000.0);
   private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);
   private final double headJointspaceWeight = 1.0;
   private final double headUserModeWeight = 1.0;
   private final double handUserModeWeight = 50.0;
   private final double handJointspaceWeight = 1.0;
   private final Vector3D handAngularTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);
   private final Vector3D handLinearTaskspaceWeight = new Vector3D(1.0, 1.0, 1.0);

   private final Vector3D headAngularWeight = new Vector3D(1.0, 1.0, 1.0);

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
      return headUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHeadJointspaceWeight()
   {
      return headJointspaceWeight;
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
      return handUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public double getHandJointspaceWeight()
   {
      return handJointspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandAngularTaskspaceWeight()
   {
      return handAngularTaskspaceWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getHandLinearTaskspaceWeight()
   {
      return handLinearTaskspaceWeight;
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
      return nContactableBodies * nContactPointsPerContactableBody * nBasisVectorsPerContactPoint;
   }

   /** @inheritDoc */
   @Override
   public TObjectDoubleHashMap<String> getJointspaceWeights()
   {
      // TODO Auto-generated method stub
      return null;
   }

   /** @inheritDoc */
   @Override
   public double getChestUserModeWeight()
   {
      return chestUserModeWeight;
   }

   /** @inheritDoc */
   @Override
   public TObjectDoubleHashMap<String> getUserModeWeights()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public Map<String, Vector3D> getTaskspaceAngularWeights()
   {
      // TODO Auto-generated method stub
      return null;
   }
}
