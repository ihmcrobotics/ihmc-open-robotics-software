package us.ihmc.stateEstimation.head.carnegie.multisense;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainUDPPacketListener;
import us.ihmc.stateEstimation.head.EKFHeadPoseEstimator;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultisenseSLWithMicroStrainHeadPoseEstimator extends EKFHeadPoseEstimator
{
   public static final RigidBodyTransform DEFAULT_MULTISENSE_TO_IMU_TRANSFORM = new RigidBodyTransform(new YawPitchRoll(0.0, Math.PI / 2.0, 0.0),
                                                                                                       new Vector3D(-0.007, -0.0524, 0.0798));
   public static final RigidBodyTransform DEFAULT_IMU_TO_MULTISENSE_TRANSFORM = new RigidBodyTransform();
   static
   {
      DEFAULT_IMU_TO_MULTISENSE_TRANSFORM.setAndInvert(DEFAULT_MULTISENSE_TO_IMU_TRANSFORM);
   }

   private static final String PARAMETER_FILE = "headPoseEstimatorTest.xml";
   private static final boolean ESTIMATE_ANGULAR_VELOCITY_BIAS = true;

   private final String simpleName = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(simpleName);

   private final MicroStrainUDPPacketListener imuListener;

   private final FullRobotModel fullRobotModel;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final FramePoint3D headPositionEstimateFromRobotModel;

   private final boolean getRobotConfigurationDataFromNetwork;

   /*
    this field is not final but is still real-time safe because
    the value returned by the UDP listener is a concurrent copier
    buffer element.
   */
   private MicroStrainData microStrainData = new MicroStrainData();

   public MultisenseSLWithMicroStrainHeadPoseEstimator(FullRobotModel fullRobotModel, double dt, RigidBodyTransform imuToHeadTransform,
                                                       PriorityParameters imuListenerPriority, long microStrainSerialNumber,
                                                       boolean getRobotConfigurationDataFromNetwork, YoVariableRegistry parentRegistry)
         throws IOException
   {
      super(dt, imuToHeadTransform, ESTIMATE_ANGULAR_VELOCITY_BIAS, parentRegistry);
      this.fullRobotModel = fullRobotModel;

      XmlParameterReader reader = new XmlParameterReader(getClass().getResourceAsStream("/" + PARAMETER_FILE));
      Set<String> defaultParameters = new HashSet<>();
      Set<String> unmatchedParameters = new HashSet<>();
      reader.readParametersInRegistry(registry, defaultParameters, unmatchedParameters);
      unmatchedParameters.forEach(p -> System.out.println("Did not find parameter " + p));

      if(imuListenerPriority != null)
      {
         imuListener = MicroStrainUDPPacketListener.createRealtimeListener(imuListenerPriority, microStrainSerialNumber);
      }
      else
      {
         imuListener = MicroStrainUDPPacketListener.createNonRealtimeListener(microStrainSerialNumber);
      }

      this.getRobotConfigurationDataFromNetwork = getRobotConfigurationDataFromNetwork;
      if (this.getRobotConfigurationDataFromNetwork)
      {
         RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                              "multisense_microstrain_head_pose_estimator");
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, ROS2Tools::generateDefaultTopicName, this::onNewDataMessage);
      }

      headPositionEstimateFromRobotModel = new FramePoint3D(ReferenceFrame.getWorldFrame());

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection)
   {
      FrameVector3D initialMagneticFieldDirection;
      if (magneticFieldDirection == null)
      {
         microStrainData = imuListener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);
         Tuple3DBasics geomagneticNorthVector = new Vector3D(microStrainData.getGeomagneticNorthVector());
         MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD.transform(geomagneticNorthVector);
         initialMagneticFieldDirection = new FrameVector3D(ReferenceFrame.getWorldFrame(), geomagneticNorthVector);
      }
      else
      {
         initialMagneticFieldDirection = magneticFieldDirection;
      }

      super.initialize(initialHeadTransform, initialMagneticFieldDirection);
   }

   @Override
   public void compute()
   {
      microStrainData = imuListener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);

      if (getRobotConfigurationDataFromNetwork)
      {
         boolean modelUpdatedWithNewData = robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null);
         if (modelUpdatedWithNewData)
         {
            headPositionEstimateFromRobotModel.set(fullRobotModel.getHeadBaseFrame().getTransformToWorldFrame().getTranslation());
            super.setEstimatedHeadPosition(headPositionEstimateFromRobotModel);
         }
      }
      else
      {
         headPositionEstimateFromRobotModel.set(fullRobotModel.getHeadBaseFrame().getTransformToWorldFrame().getTranslation());
         super.setEstimatedHeadPosition(headPositionEstimateFromRobotModel);
      }

      if(microStrainData != null)
      {
         super.setImuAngularVelocity(microStrainData.getAngularRate());
         super.setImuLinearAcceleration(microStrainData.getLinearAcceleration());
         super.setImuMagneticFieldVector(microStrainData.getGeomagneticNorthVector());
      }

      super.compute();
   }

   private void updateRobotConfigurationData(RobotConfigurationData latestData)
   {
      robotConfigurationDataBuffer.receivedPacket(latestData);
   }

   private void onNewDataMessage(Subscriber<RobotConfigurationData> subscriber)
   {
      updateRobotConfigurationData(subscriber.takeNextData());
   }
}
