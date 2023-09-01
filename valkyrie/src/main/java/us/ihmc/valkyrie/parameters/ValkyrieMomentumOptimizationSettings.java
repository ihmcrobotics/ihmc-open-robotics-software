package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ValkyrieMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // defaults for unscaled model:
   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 4.0;
   private static final double defaultRhoRateDefaultWeight = 5E-8;
   private static final double defaultRhoRateHighWeight = 8.0E-7;

   private final Vector3D linearMomentumWeight = new Vector3D(0.05, 0.05, 0.01);
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.1);

   private final Vector3D footAngularWeight = new Vector3D(0.5, 0.5, 0.5);
   private final Vector3D footLinearWeight = new Vector3D(30.0, 30.0, 30.0);
   private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D pelvisAngularWeight = new Vector3D(5.0, 5.0, 5.0);
   private final Vector3D pelvisLinearWeight = new Vector3D(5.0, 5.0, 30.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies = 2;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 1.6E-6;
   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;
   private final Vector2D copWeight = new Vector2D(200.0, 200.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(0.32, 0.32);
   private final Vector2D copRateHighWeight = new Vector2D(0.0004, 0.0016);

   private final double neckJointspaceWeight = 5.0;
   private final double spineJointspaceWeight = 10.0;
   private final double armJointspaceWeight = 1.0;
   private final double legJointspaceWeight = 1.0;
   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   private final double neckUserModeWeight = 50.0;
   private final double spineUserModeWeight = 50.0;
   private final double armUserModeWeight = 50.0;
   private final double legUserModeWeight = 50.0;
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   private final Vector3D headAngularWeight = new Vector3D(500.0, 500.0, 500.0);
   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final Vector3D handAngularWeight = new Vector3D(0.5, 0.5, 0.5);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   private final Vector3D handLinearWeight = new Vector3D(5.0, 5.0, 5.0);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public ValkyrieMomentumOptimizationSettings(ValkyrieJointMap jointMap)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      linearMomentumWeight.scale(1.0 / scale);
      angularMomentumWeight.scale(1.0 / scale);
      rhoWeight = defaultRhoWeight / scale;
      rhoMin = defaultRhoMin * scale;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      jointspaceWeights.add(new GroupParameter<>("Spine", spineJointspaceWeight, jointMap.getSpineJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Arms", armJointspaceWeight, jointMap.getArmJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Legs", legJointspaceWeight, jointMap.getLegJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Neck", neckJointspaceWeight, jointMap.getNeckJointNamesAsStrings()));

      userModeWeights.add(new GroupParameter<>("Spine", spineUserModeWeight, jointMap.getSpineJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Arms", armUserModeWeight, jointMap.getArmJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Legs", legUserModeWeight, jointMap.getLegJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Neck", neckUserModeWeight, jointMap.getNeckJointNamesAsStrings()));

      taskspaceAngularWeights.add(new GroupParameter<>("Chest", chestAngularWeight, Collections.singletonList(jointMap.getChestName())));
      taskspaceAngularWeights.add(new GroupParameter<>("Head", headAngularWeight, Collections.singletonList(jointMap.getHeadName())));

      taskspaceAngularWeights.add(new GroupParameter<>("Pelvis", pelvisAngularWeight, Collections.singletonList(jointMap.getPelvisName())));
      taskspaceLinearWeights.add(new GroupParameter<>("Pelvis", pelvisLinearWeight, Collections.singletonList(jointMap.getPelvisName())));

      List<String> handNames = jointMap.getHandNames();
      List<String> footNames = jointMap.getFootNames();
      taskspaceAngularWeights.add(new GroupParameter<>("Hand", handAngularWeight, handNames));
      taskspaceLinearWeights.add(new GroupParameter<>("Hand", handLinearWeight, handNames));
      taskspaceAngularWeights.add(new GroupParameter<>("Foot", footAngularWeight, footNames));
      taskspaceLinearWeights.add(new GroupParameter<>("Foot", footLinearWeight, footNames));
   }

   /** @inheritDoc */
   @Override
   public Vector3D getLinearMomentumWeight()
   {
      return linearMomentumWeight;
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
   public Vector3D getLoadedFootLinearWeight()
   {
      return highLinearFootWeight;
   }

   /** @inheritDoc */
   @Override
   public Vector3D getLoadedFootAngularWeight()
   {
      return highAngularFootWeight;
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
   public List<GroupParameter<Double>> getJointspaceWeights()
   {
      return jointspaceWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Double>> getUserModeWeights()
   {
      return userModeWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceAngularWeights()
   {
      return taskspaceAngularWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceLinearWeights()
   {
      return taskspaceLinearWeights;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useWarmStartInSolver()
   {
      return true;
   }
}
