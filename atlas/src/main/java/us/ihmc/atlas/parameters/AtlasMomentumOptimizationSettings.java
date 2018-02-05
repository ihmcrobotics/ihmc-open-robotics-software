package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // defaults for unscaled model:
   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 4.0;
   private static final double defaultRhoRateDefaultWeight = 0.002;
   private static final double defaultRhoRateHighWeight = 0.05;

   private static final boolean useWarmStartInSolver = true;
   private static final boolean disableRhosWhenNotInContact = true;

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = AtlasContactPointParameters.USE_SIX_CONTACT_POINTS ? 6 : 4;
   private final int nContactableBodies;

   private final double jointAccelerationWeight = 0.005;
   private final double jointJerkWeight = 0.1;
   private final double jointTorqueWeight = 0.005;
   private final Vector2D copWeight = new Vector2D(100.0, 200.0);
   private final Vector2D copRateDefaultWeight = new Vector2D(20000.0, 20000.0);
   private final Vector2D copRateHighWeight = new Vector2D(2500000.0, 10000000.0);

   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;

   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public AtlasMomentumOptimizationSettings(DRCRobotJointMap jointMap, int numberOfContactableBodies)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      rhoWeight = defaultRhoWeight / scale;
      rhoMin = defaultRhoMin * scale;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      userModeWeights.add(new GroupParameter<>("Spine", jointMap.getSpineJointNamesAsStrings()));
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_YAW);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_PITCH);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_ROLL);

      jointspaceWeights.add(new GroupParameter<>("Arms", jointMap.getArmJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Arms", jointMap.getArmJointNamesAsStrings()));

      jointspaceWeights.add(new GroupParameter<>("Neck", jointMap.getNeckJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Neck", jointMap.getNeckJointNamesAsStrings()));

      taskspaceAngularWeights.add(new GroupParameter<>("Chest", jointMap.getChestName()));
      taskspaceAngularWeights.add(new GroupParameter<>("Head", jointMap.getHeadName()));

      taskspaceAngularWeights.add(new GroupParameter<>("Pelvis", jointMap.getPelvisName()));
      taskspaceLinearWeights.add(new GroupParameter<>("Pelvis", jointMap.getPelvisName()));

      List<String> handNames = jointMap.getHandNames();
      List<String> footNames = jointMap.getFootNames();
      taskspaceAngularWeights.add(new GroupParameter<>("Hand", handNames));
      taskspaceLinearWeights.add(new GroupParameter<>("Hand", handNames));
      taskspaceAngularWeights.add(new GroupParameter<>("Foot", footNames));
      taskspaceLinearWeights.add(new GroupParameter<>("Foot", footNames));

      this.nContactableBodies = numberOfContactableBodies;
   }

   private static void configureBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, SpineJointName jointName)
   {
      behaviors.add(new GroupParameter<>(jointName.toString(), jointMap.getSpineJointName(jointName)));
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
   public double getJointTorqueWeight()
   {
      return jointTorqueWeight;
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

   /** @inheritDoc */
   @Override
   public boolean useWarmStartInSolver()
   {
      return useWarmStartInSolver;
   }

   @Override
   public boolean getDeactivateRhoWhenNotInContact()
   {
      return disableRhosWhenNotInContact;
   }
}
