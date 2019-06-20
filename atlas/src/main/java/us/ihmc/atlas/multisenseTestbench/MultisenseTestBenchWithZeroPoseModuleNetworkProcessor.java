package us.ihmc.atlas.multisenseTestbench;

import java.net.URI;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import org.ros.message.Time;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.ros.publisher.RosOdometryPublisher;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultisenseTestBenchWithZeroPoseModuleNetworkProcessor implements PacketConsumer<RobotConfigurationData>, Runnable
{
   private static final String DEFAULT_ROS_NAMESPACE = "/ihmc_ros/atlas";
   private static final String NODE_NAME = "/multisense_testbench";
   public static final String WORLD_FRAME_NAME = "world";

   private final String rosNamespace;
   private final RosMainNode rosMainNode;
   private final RosJointStatePublisher rosJointStatePublisher;
   private final RosOdometryPublisher pelvisOdometryPublisher;
   private final RosTfPublisher tfPublisher;

   private final ArrayList<String> jointNamesList;
   private final FullHumanoidRobotModel fullRobotModel;
   private final DRCRobotModel robotModel;
   private final int jointNameHash;


   private final ArrayBlockingQueue<RobotConfigurationData> availableRobotConfigurationData = new ArrayBlockingQueue<RobotConfigurationData>(30);
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider = new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   private final DRCRobotJointMap jointMap;

   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTransforms;

   public MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(DRCRobotModel robotModel, String rosNamespace)
   {
      this.robotModel = robotModel;
      URI rosUri = NetworkParameters.getROSURI();

      DRCNetworkModuleParameters params = new DRCNetworkModuleParameters();

      params.setRosUri(rosUri);
      params.enableRosModule(true);
      params.enableSensorModule(true);
      params.enableUiModule(true);
      params.enableZeroPoseRobotConfigurationPublisherModule(true);

      DRCNetworkProcessor drcNetworkProcessor = new DRCNetworkProcessor(robotModel, params);

      if (rosNamespace == null || rosNamespace.isEmpty())
      {
         this.rosNamespace = DEFAULT_ROS_NAMESPACE;
      }
      else
      {
         this.rosNamespace = rosNamespace;
      }

      PacketCommunicator rosAPICommunicator = PacketCommunicator
            .createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

      staticTransforms = robotModel.getSensorInformation().getStaticTransformsForRos();

      rosMainNode = new RosMainNode(rosUri, this.rosNamespace + NODE_NAME);

      rosJointStatePublisher = new RosJointStatePublisher(false);

      fullRobotModel = robotModel.createFullRobotModel();
      OneDoFJointBasics[] controllableOneDoFJoints = fullRobotModel.getControllableOneDoFJoints();

      jointNamesList = new ArrayList<>(controllableOneDoFJoints.length);

      for (int i = 0; i < controllableOneDoFJoints.length; i++)
      {
         jointNamesList.add(controllableOneDoFJoints[i].getName());
      }

      jointNameHash = RobotConfigurationDataFactory
            .calculateJointNameHash(controllableOneDoFJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

      rosAPICommunicator.attachListener(RobotConfigurationData.class, this);

      pelvisOdometryPublisher = new RosOdometryPublisher(false);

      tfPublisher = new RosTfPublisher(rosMainNode, null);

      rosMainNode.attachPublisher(rosNamespace + "/output/joint_states", rosJointStatePublisher);
      rosMainNode.attachPublisher(rosNamespace + "/output/robot_pose", pelvisOdometryPublisher);

      rosMainNode.execute();
      jointMap = robotModel.getJointMap();
   }

   public MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(DRCRobotModel robotModel)
   {
      this(robotModel, DEFAULT_ROS_NAMESPACE);
   }

   public static void main(String[] args) throws InterruptedException
   {
      AtlasRobotModel robotModel = AtlasRobotModelFactory
            .createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS.name(), RobotTarget.REAL_ROBOT, false);

      Thread t = new Thread(new MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(robotModel));

      t.start();

      t.join();
   }

   @Override
   public void receivedPacket(RobotConfigurationData robotConfigurationData)
   {
      if(!availableRobotConfigurationData.offer(robotConfigurationData))
      {
         availableRobotConfigurationData.clear();
      }
   }

   @Override
   public void run()
   {
      while(true)
      {
         RobotConfigurationData robotConfigurationData;
         try
         {
            robotConfigurationData = availableRobotConfigurationData.take();
         }
         catch (InterruptedException e)
         {
            // Ignore and skip to the next loop iteration
            continue;
         }
         if (rosMainNode.isStarted())
         {
            float[] jointAngles = robotConfigurationData.getJointAngles().toArray();
            float[] jointVelocities = robotConfigurationData.getJointVelocities().toArray();
            float[] jointTorques = robotConfigurationData.getJointTorques().toArray();

            long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(robotConfigurationData.getMonotonicTime());
            Time t = Time.fromNano(timeStamp);

            if (robotConfigurationData.getJointNameHash() != jointNameHash)
            {
               throw new RuntimeException("Joint names do not match for RobotConfigurationData");
            }

            rosJointStatePublisher.publish(jointNamesList, jointAngles, jointVelocities, jointTorques, t);

            RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());

            pelvisOdometryPublisher.publish(timeStamp, pelvisTransform, robotConfigurationData.getPelvisLinearVelocity(),
                                            robotConfigurationData.getPelvisAngularVelocity(), jointMap.getUnsanitizedRootJointInSdf(),
                                            WORLD_FRAME_NAME);

            tfPublisher.publish(pelvisTransform, timeStamp, WORLD_FRAME_NAME, jointMap.getUnsanitizedRootJointInSdf());

            if(staticTransforms != null)
            {
               for (int i = 0; i < staticTransforms.size(); i++)
               {
                  ImmutableTriple<String, String, RigidBodyTransform> staticTransformTriplet = staticTransforms.get(i);
                  String from = staticTransformTriplet.getLeft();
                  String to = staticTransformTriplet.getMiddle();
                  RigidBodyTransform staticTransform = staticTransformTriplet.getRight();
                  tfPublisher.publish(staticTransform, timeStamp, from, to);
               }
            }
         }
      }
   }
}
