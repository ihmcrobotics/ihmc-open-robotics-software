package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieMomentumOptimizationSettings extends MomentumOptimizationSettings
{
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
   private final double rhoWeight = 0.00001;
   private final double rhoMin = 4.0;
   private final double rhoRateDefaultWeight = 5E-8;
   private final double rhoRateHighWeight = 8.0E-7;
   private final Vector2D copWeight = new Vector2D(200.0, 200.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(0.32, 0.32);
   private final Vector2D copRateHighWeight = new Vector2D(0.0004, 0.0016);

   private final double neckJointspaceWeight = 5.0;
   private final double spineJointspaceWeight = 10.0;
   private final double armJointspaceWeight = 1.0;
   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   private final double neckUserModeWeight = 50.0;
   private final double spineUserModeWeight = 50.0;
   private final double armUserModeWeight = 50.0;
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   private final Vector3D headAngularWeight = new Vector3D(500.0, 500.0, 500.0);
   private final Vector3D chestAngularWeight = new Vector3D(15.0, 10.0, 5.0);
   private final Vector3D handAngularWeight = new Vector3D(0.5, 0.5, 0.5);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   private final Vector3D handLinearWeight = new Vector3D(5.0, 5.0, 5.0);
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public ValkyrieMomentumOptimizationSettings(ValkyrieJointMap jointMap)
   {
      for (SpineJointName jointName : jointMap.getSpineJointNames())
      {
         configureBehavior(jointspaceWeights, jointMap, jointName, spineJointspaceWeight);
         configureBehavior(userModeWeights, jointMap, jointName, spineUserModeWeight);
      }

      for (ArmJointName jointName : jointMap.getArmJointNames())
      {
         configureSymmetricBehavior(jointspaceWeights, jointMap, jointName, armJointspaceWeight);
         configureSymmetricBehavior(userModeWeights, jointMap, jointName, armUserModeWeight);
      }

      for (NeckJointName jointName : jointMap.getNeckJointNames())
      {
         configureBehavior(jointspaceWeights, jointMap, jointName, neckJointspaceWeight);
         configureBehavior(userModeWeights, jointMap, jointName, neckUserModeWeight);
      }

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

   private static void configureSymmetricBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, ArmJointName jointName, double weight)
   {
      behaviors.add(new GroupParameter<>(jointName.toString(), new Double(weight), jointMap.getLeftAndRightJointNames(jointName)));
   }

   private static void configureBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, SpineJointName jointName, double weight)
   {
      List<String> names = Collections.singletonList(jointMap.getSpineJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), new Double(weight), names));
   }

   private static void configureBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, NeckJointName jointName, double weight)
   {
      List<String> names = Collections.singletonList(jointMap.getNeckJointName(jointName));
      behaviors.add(new GroupParameter<>(jointName.toString(), new Double(weight), names));
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
