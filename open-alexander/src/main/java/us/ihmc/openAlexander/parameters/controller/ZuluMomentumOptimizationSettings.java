package us.ihmc.openAlexander.parameters.controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ZuluMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   // TODO Parameter values copied from Valkyrie which has different weight.
   // TODO Needs tune up.
   private final Vector3D footAngularWeight = new Vector3D(0.5, 0.5, 0.5);
   // TODO Needs tune up.
   private final Vector3D footLinearWeight = new Vector3D(30.0, 30.0, 30.0);

   private final Vector3D pelvisAngularWeight = new Vector3D(10.0, 10.0, 3.0);
   // TODO Needs tune up.
   private final Vector3D pelvisLinearWeight = new Vector3D(5.0, 5.0, 30.0);

   private final int nContactableBodies;

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
      armJointspaceWeight = OpenAlexanderWalkingControllerParameters.RESPONSIVE_STREAMING_MODE ? 3.0 : 1.0;

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
}
