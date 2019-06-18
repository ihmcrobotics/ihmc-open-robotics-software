package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * A wrapper class for the {@link LeggedRobotEKF}: for parts of the upper body (such as the arms) we do not want to use
 * the EKF to save some computation time. The joints that are not passed to the EKF are the "simple joints" their
 * measurements are set directly from the processed sensor output.
 *
 * @author Georg Wiedebach
 */
public class HumanoidRobotEKFWithSimpleJoints implements StateEstimatorController
{
   private final SensorOutputMapReadOnly processedSensorOutput;
   private final List<OneDoFJoint> simpleJoints;

   /**
    * The reference joints are needed for now since the estimator operates on a copy of the actual robot model used. To
    * get the measurements from the sensor map the queries have to be made using the reference joints.
    * <p>
    * TODO: once the EKF is ready to be run as an alternative to the DRC estimator the referenceModel should be removed.
    */
   private final List<OneDoFJoint> referenceJoints;

   private final LeggedRobotEKF leggedRobotEKF;

   public HumanoidRobotEKFWithSimpleJoints(FullHumanoidRobotModel estimatorFullRobotModel, String primaryImuName, Collection<String> imuNames,
                                           SideDependentList<String> footForceSensorNames, SensorRawOutputMapReadOnly rawSensorOutput, double dt,
                                           double gravity, SensorOutputMapReadOnly processedSensorOutput, YoGraphicsListRegistry graphicsListRegistry,
                                           FullHumanoidRobotModel referenceModel)
   {
      this.processedSensorOutput = processedSensorOutput;

      simpleJoints = SubtreeStreams.fromChildren(OneDoFJoint.class, estimatorFullRobotModel.getChest()).collect(Collectors.toList());
      referenceJoints = SubtreeStreams.fromChildren(OneDoFJoint.class, referenceModel.getChest()).collect(Collectors.toList());

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

      Map<String, ImmutablePair<ReferenceFrame, ForceSensorDefinition>> forceSensorMap = new HashMap<>();
      List<ForceSensorDefinition> forceSensorDefinitions = Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions());
      for (RobotSide robotSide : RobotSide.values)
      {
         String sensorName = footForceSensorNames.get(robotSide);
         ForceSensorDefinition sensorDefinition = forceSensorDefinitions.stream().filter(d -> d.getSensorName().equals(sensorName)).findFirst().get();
         MovingReferenceFrame soleFrame = estimatorFullRobotModel.getSoleFrame(robotSide);
         forceSensorMap.put(sensorName, new ImmutablePair<>(soleFrame, sensorDefinition));
      }

      Map<String, IMUDefinition> imuSensorMap = new HashMap<>();
      List<IMUDefinition> imuSensorDefinitions = Arrays.asList(estimatorFullRobotModel.getIMUDefinitions());
      for (String imuName : imuNames)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.stream().filter(d -> d.getName().equals(imuName)).findFirst().get();
         imuSensorMap.put(imuName, imuDefinition);
      }

      Map<String, String> jointParameterGroups = createJointGroups(estimatorFullRobotModel);

      FloatingJointBasics rootJoint = estimatorFullRobotModel.getRootJoint();
      leggedRobotEKF = new LeggedRobotEKF(rootJoint, jointsForEKF, primaryImuName, imuSensorMap, forceSensorMap, rawSensorOutput, processedSensorOutput, dt,
                                          gravity, jointParameterGroups, graphicsListRegistry, referenceJointsForEKF);
   }

   private static Map<String, String> createJointGroups(FullHumanoidRobotModel fullRobotModel)
   {
      Map<String, String> ret = new HashMap<>();

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      OneDoFJointBasics[] spineJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, fullRobotModel.getChest());
      Arrays.asList(spineJoints).forEach(joint -> ret.put(joint.getName(), "Spine"));
      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, fullRobotModel.getFoot(robotSide));
         Arrays.asList(legJoints).forEach(joint -> ret.put(joint.getName(), "Leg"));
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
