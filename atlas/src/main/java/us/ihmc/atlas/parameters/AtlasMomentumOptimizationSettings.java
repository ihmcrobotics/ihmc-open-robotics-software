package us.ihmc.atlas.parameters;

import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // defaults for unscaled model:
   private static final double defaultRhoWeight = 0.00001;
   private static final double defaultRhoMin = 4.0;
   private static final double defaultRhoRateDefaultWeight = 3.2E-8;
   private static final double defaultRhoRateHighWeight = 8.0E-7;

   private static final boolean useWarmStartInSolver = true;
   private static final boolean disableRhosWhenNotInContact = true;

   private final int nBasisVectorsPerContactPoint = 4;
   private final int nContactPointsPerContactableBody = AtlasContactPointParameters.USE_SIX_CONTACT_POINTS ? 6 : 4;
   private final int nContactableBodies;

   private final double jointAccelerationWeight = 0.01;
   private final double jointJerkWeight = 0.0;
   private final double jointTorqueWeight = 0.005;
   private final Vector2D copWeight = new Vector2D(0.001, 0.002);
   private final Vector2D copRateDefaultWeight = new Vector2D(0.00000032, 0.00000032);
   private final Vector2D copRateHighWeight = new Vector2D(0.00004, 0.00016);

   private final double rhoWeight;
   private final double rhoMin;
   private final double rhoRateDefaultWeight;
   private final double rhoRateHighWeight;

   private final List<GroupParameter<Double>> jointspaceWeightGroups = new ArrayList<>();
   private final List<GroupParameter<Double>> userModeWeightGroups = new ArrayList<>();
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeightGroups = new ArrayList<>();
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeightGroups = new ArrayList<>();

   public AtlasMomentumOptimizationSettings(DRCRobotJointMap jointMap, int numberOfContactableBodies)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      rhoWeight = defaultRhoWeight / scale;
      rhoMin = defaultRhoMin * scale;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      userModeWeightGroups.add(new GroupParameter<>("Spine", jointMap.getSpineJointNamesAsStrings()));
      jointspaceWeightGroups.add(new GroupParameter<>(SPINE_YAW.toString(), jointMap.getSpineJointName(SPINE_YAW)));
      jointspaceWeightGroups.add(new GroupParameter<>(SPINE_PITCH.toString(), jointMap.getSpineJointName(SPINE_PITCH)));
      jointspaceWeightGroups.add(new GroupParameter<>(SPINE_ROLL.toString(), jointMap.getSpineJointName(SPINE_ROLL)));

      jointspaceWeightGroups.add(new GroupParameter<>("Arms", jointMap.getArmJointNamesAsStrings()));
      userModeWeightGroups.add(new GroupParameter<>("Arms", jointMap.getArmJointNamesAsStrings()));

      jointspaceWeightGroups.add(new GroupParameter<>("Neck", jointMap.getNeckJointNamesAsStrings()));
      userModeWeightGroups.add(new GroupParameter<>("Neck", jointMap.getNeckJointNamesAsStrings()));

      taskspaceAngularWeightGroups.add(new GroupParameter<>("Chest", jointMap.getChestName()));
      taskspaceAngularWeightGroups.add(new GroupParameter<>("Head", jointMap.getHeadName()));

      taskspaceAngularWeightGroups.add(new GroupParameter<>("Pelvis", jointMap.getPelvisName()));
      taskspaceLinearWeightGroups.add(new GroupParameter<>("Pelvis", jointMap.getPelvisName()));

      List<String> handNames = jointMap.getHandNames();
      List<String> footNames = jointMap.getFootNames();
      taskspaceAngularWeightGroups.add(new GroupParameter<>("Hand", handNames));
      taskspaceLinearWeightGroups.add(new GroupParameter<>("Hand", handNames));
      taskspaceAngularWeightGroups.add(new GroupParameter<>("Foot", footNames));
      taskspaceLinearWeightGroups.add(new GroupParameter<>("Foot", footNames));

      this.nContactableBodies = numberOfContactableBodies;
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
   public List<GroupParameter<Double>> getJointspaceWeights()
   {
      return jointspaceWeightGroups;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Double>> getUserModeWeights()
   {
      return userModeWeightGroups;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceAngularWeights()
   {
      return taskspaceAngularWeightGroups;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceLinearWeights()
   {
      return taskspaceLinearWeightGroups;
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
