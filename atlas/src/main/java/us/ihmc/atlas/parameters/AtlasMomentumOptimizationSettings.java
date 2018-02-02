package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   private final Vector3D linearMomentumWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D highLinearMomentumWeightForRecovery = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D angularMomentumWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here

   private final Vector3D footAngularWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D footLinearWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D highAngularFootWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D highLinearFootWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here

   private final Vector3D pelvisAngularWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D pelvisLinearWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here

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

   private final double neckJointspaceWeight = Double.NaN; // do not tune here
   private final double spineJointspaceWeightYaw = Double.NaN; // do not tune here
   private final double spineJointspaceWeightPitch = Double.NaN; // do not tune here
   private final double spineJointspaceWeightRoll = Double.NaN; // do not tune here
   private final double armJointspaceWeight = Double.NaN; // do not tune here
   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   private final double neckUserModeWeight = Double.NaN; // do not tune here
   private final double spineUserModeWeight = Double.NaN; // do not tune here
   private final double armUserModeWeight = Double.NaN; // do not tune here
   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   private final Vector3D headAngularWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D chestAngularWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final Vector3D handAngularWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   private final Vector3D handLinearWeight = new Vector3D(Double.NaN, Double.NaN, Double.NaN); // do not tune here
   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public AtlasMomentumOptimizationSettings(DRCRobotJointMap jointMap, int numberOfContactableBodies)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      rhoWeight = defaultRhoWeight / scale;
      rhoMin = defaultRhoMin * scale;
      rhoRateDefaultWeight = defaultRhoRateDefaultWeight / (scale * scale);
      rhoRateHighWeight = defaultRhoRateHighWeight / (scale * scale);

      linearMomentumWeight.scale(1.0 / scale);
      highLinearMomentumWeightForRecovery.scale(1.0 / scale);
      angularMomentumWeight.scale(1.0 / scale);

      userModeWeights.add(new GroupParameter<>("Spine", spineUserModeWeight, jointMap.getSpineJointNamesAsStrings()));
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_YAW, spineJointspaceWeightYaw);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_PITCH, spineJointspaceWeightPitch);
      configureBehavior(jointspaceWeights, jointMap, SpineJointName.SPINE_ROLL, spineJointspaceWeightRoll);

      jointspaceWeights.add(new GroupParameter<>("Arms", armJointspaceWeight, jointMap.getArmJointNamesAsStrings()));
      userModeWeights.add(new GroupParameter<>("Arms", armUserModeWeight, jointMap.getArmJointNamesAsStrings()));

      jointspaceWeights.add(new GroupParameter<>("Neck", neckJointspaceWeight, jointMap.getNeckJointNamesAsStrings()));
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

      this.nContactableBodies = numberOfContactableBodies;
   }

   private static void configureBehavior(List<GroupParameter<Double>> behaviors, DRCRobotJointMap jointMap, SpineJointName jointName, double weight)
   {
      List<String> names = Collections.singletonList(jointMap.getSpineJointName(jointName));
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
