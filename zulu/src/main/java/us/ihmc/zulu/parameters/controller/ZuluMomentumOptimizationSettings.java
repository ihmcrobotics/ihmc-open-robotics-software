package us.ihmc.zulu.parameters.controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ZuluMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // TODO Parameter values copied from Valkyrie which has different weight.
   // TODO Needs tune up.
   private final Vector3D linearMomentumWeight = new Vector3D(0.5, 0.5, 0.02);
   // TODO The z-component is super high, probably causing tracking issues in swing
   private final Vector3D angularMomentumWeight = new Vector3D(0.0, 0.0, 0.1);

   // TODO Needs tune up.
   private final Vector3D footAngularWeight = new Vector3D(0.5, 0.5, 0.5);
   // TODO Needs tune up.
   private final Vector3D footLinearWeight = new Vector3D(30.0, 30.0, 30.0);
   // TODO Needs tune up.
   private final Vector3D highAngularFootWeight = new Vector3D(5.0, 5.0, 5.0);
   // TODO Needs tune up.
   private final Vector3D highLinearFootWeight = new Vector3D(50.0, 50.0, 50.0);

   private final Vector3D pelvisAngularWeight = new Vector3D(10.0, 10.0, 3.0);
   // TODO Needs tune up.
   private final Vector3D pelvisLinearWeight = new Vector3D(5.0, 5.0, 30.0);

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = 4;
   private final int nContactableBodies;

   // TODO Needs tune up.
   private final double jointAccelerationWeight = 0.005;
   // TODO Needs tune up.
   private final double jointJerkWeight = 1.6E-6;
   private final double rhoWeight = 5e-6;
   // TODO Needs tune up.
   private final double rhoMin = 2.0;
   private final double rhoRateDefaultWeight = 1E-8;
   // TODO Needs tune up.
   private final double rhoRateHighWeight = 1.6E-6;
   // TODO Needs tune up.
   private final Vector2D copWeight = new Vector2D(0.002, 0.004);
   // TODO Needs tune up.
   private final Vector2D copRateDefaultWeight = new Vector2D(0.00000064, 0.00000064);
   // TODO Needs tune up.
   private final Vector2D copRateHighWeight = new Vector2D(0.00008, 0.00032);

   private final double neckJointspaceWeight = 50.0;
   // TODO Needs tune up.
   private final double spineJointspaceWeight = 10.0;
   // TODO Needs tune up.
   private final double armJointspaceWeight;
   // TODO Needs tune up.
   private final double legJointspaceWeight = 1.0;
   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   // We don't really use this feature, bundling all joints together.
   private final double jointUserModeWeight = 50.0;
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   // TODO Needs tune up.
   private final Vector3D headAngularWeight = new Vector3D(500.0, 500.0, 500.0);
   // TODO Needs tune up.
   private final Vector3D chestAngularWeight = new Vector3D(30.0, 30.0, 5.0);

   private final Vector3D handAngularWeight;

   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   // TODO Needs tune up.
   private final Vector3D handLinearWeight = new Vector3D(5.0, 5.0, 5.0);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public ZuluMomentumOptimizationSettings(RobotTarget target, HumanoidJointNameMap jointMap)
   {
      this(target, jointMap, 2);
   }

   public ZuluMomentumOptimizationSettings(RobotTarget target, HumanoidJointNameMap jointMap, int numberOfContactableBodies)
   {
      List<String> neckNames = jointMap.getNeckJointNamesAsStrings();
      List<String> spineNames = jointMap.getSpineJointNamesAsStrings();
      List<String> armNames = jointMap.getArmJointNamesAsStrings();
      List<String> legNames = jointMap.getLegJointNamesAsStrings();

      List<String> allUserModeJoints = new ArrayList<>();
      allUserModeJoints.addAll(neckNames);
      allUserModeJoints.addAll(spineNames);
      allUserModeJoints.addAll(armNames);
      allUserModeJoints.addAll(legNames);

      if (jointMap.getArmJointNames().length == 4)
      { // For 4-dof arm, only control Z orientation of hand and place at higher weight
         handAngularWeight = new Vector3D(0.0, 0.0, 0.5);
      }
      else
      { // Full 7dof arm mode
         handAngularWeight = new Vector3D(1.0, 1.0, 1.0);
      }

      userModeWeights.add(new GroupParameter<Double>("allJoints", jointUserModeWeight, allUserModeJoints));
      armJointspaceWeight = ZuluWalkingControllerParameters.RESPONSIVE_STREAMING_MODE ? 3.0 : 1.0;

      jointspaceWeights.add(new GroupParameter<>("Spine", spineJointspaceWeight, jointMap.getSpineJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Arms", armJointspaceWeight, jointMap.getArmJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Legs", legJointspaceWeight, jointMap.getLegJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Neck", neckJointspaceWeight, jointMap.getNeckJointNamesAsStrings()));

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

      nContactableBodies = numberOfContactableBodies;
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
