package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HumanoidRobotEKFWithSimpleJoints implements StateEstimatorController
{
   private final SensorOutputMapReadOnly processedSensorOutput;
   private final List<OneDoFJoint> simpleJoints;
   private final List<OneDoFJoint> referenceJoints;

   private final LeggedRobotEKF leggedRobotEKF;

   public HumanoidRobotEKFWithSimpleJoints(FullHumanoidRobotModel estimatorFullRobotModel, String primaryImuName, Collection<String> imuNames,
                                           SideDependentList<String> footForceSensorNames, SensorRawOutputMapReadOnly rawSensorOutput, double dt, double gravity,
                                           SensorOutputMapReadOnly processedSensorOutput, YoGraphicsListRegistry graphicsListRegistry,
                                           FullHumanoidRobotModel referenceModel)
   {
      this.processedSensorOutput = processedSensorOutput;

      JointBasics[] chestSubtreeJoints = MultiBodySystemTools.collectSubtreeJoints(estimatorFullRobotModel.getChest());
      simpleJoints = Arrays.asList(MultiBodySystemTools.filterJoints(chestSubtreeJoints, OneDoFJoint.class));
      if (simpleJoints.size() != chestSubtreeJoints.length)
      {
         throw new RuntimeException("Can only handle OneDoFJoints in a robot.");
      }

      JointBasics[] referenceChestSubtreeJoints = MultiBodySystemTools.collectSubtreeJoints(referenceModel.getChest());
      referenceJoints = Arrays.asList(MultiBodySystemTools.filterJoints(referenceChestSubtreeJoints, OneDoFJoint.class));

      List<OneDoFJointBasics> jointsForEKF = new ArrayList<>();
      List<OneDoFJointBasics> referenceJointsForEKF = new ArrayList<>();
      for (OneDoFJointBasics oneDoFJoint : estimatorFullRobotModel.getOneDoFJoints())
      {
         if (!simpleJoints.contains(oneDoFJoint))
         {
            jointsForEKF.add(oneDoFJoint);
            referenceJointsForEKF.add(referenceModel.getOneDoFJointByName(oneDoFJoint.getName()));
         }
      }

      Map<String, ReferenceFrame> forceSensorMap = new HashMap<>();
      SideDependentList<MovingReferenceFrame> soleFrames = estimatorFullRobotModel.getSoleFrames();
      for (RobotSide robotSide : RobotSide.values)
      {
         forceSensorMap.put(footForceSensorNames.get(robotSide), soleFrames.get(robotSide));
      }

      Map<String, String> jointParameterGroups = createJointGroups(estimatorFullRobotModel);

      FloatingJointBasics rootJoint = estimatorFullRobotModel.getRootJoint();
      leggedRobotEKF = new LeggedRobotEKF(rootJoint, jointsForEKF, primaryImuName, imuNames, forceSensorMap, rawSensorOutput, processedSensorOutput, dt, gravity,
                                          jointParameterGroups, graphicsListRegistry, referenceJointsForEKF);
   }

   private static Map<String, String> createJointGroups(FullHumanoidRobotModel fullRobotModel)
   {
      Map<String, String> ret = new HashMap<>();

      RigidBodyBasics leftBody = fullRobotModel.getFoot(RobotSide.LEFT);
      RigidBodyBasics rightBody = fullRobotModel.getFoot(RobotSide.RIGHT);
      while (leftBody != rightBody)
      {
         String leftName = leftBody.getParentJoint().getName();
         String rightName = rightBody.getParentJoint().getName();
         String name = "";
         int leftIdx = leftName.length() - 1;
         for (int rightIdx = rightName.length() - 1; rightIdx >= 0; rightIdx--)
         {
            if (leftIdx < 0)
            {
               break;
            }
            if (rightName.charAt(rightIdx) == leftName.charAt(leftIdx))
            {
               name = rightName.charAt(rightIdx) + name;
            }
            else
            {
               break;
            }
            leftIdx--;
         }
         ret.put(leftName, name);
         ret.put(rightName, name);
         leftBody = leftBody.getParentJoint().getPredecessor();
         rightBody = rightBody.getParentJoint().getPredecessor();
      }

      return ret;
   }

   @Override
   public void doControl()
   {
      for (int jointIdx = 0; jointIdx < simpleJoints.size(); jointIdx++)
      {
         OneDoFJoint simpleJoint = simpleJoints.get(jointIdx);
         OneDoFJoint referenceJoint = referenceJoints.get(jointIdx);
         simpleJoint.setQ(processedSensorOutput.getJointPositionProcessedOutput(referenceJoint));
         simpleJoint.setQd(processedSensorOutput.getJointVelocityProcessedOutput(referenceJoint));
      }

      leggedRobotEKF.doControl();
   }

   @Override
   public void initialize()
   {
      leggedRobotEKF.initialize();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return leggedRobotEKF.getYoVariableRegistry();
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      leggedRobotEKF.initializeEstimator(rootJointTransform, jointPositions);
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      leggedRobotEKF.requestStateEstimatorMode(operatingMode);
   }
}
