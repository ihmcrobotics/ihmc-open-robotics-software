package us.ihmc.stateEstimation.head.carnegie.multisense;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.YoPoint3D;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainUDPPacketListener;
import us.ihmc.stateEstimation.head.EKFHeadPoseEstimator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.io.IOException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultisenseSLWithMicroStrainHeadPoseEstimator extends EKFHeadPoseEstimator
{
   private static final boolean USE_VIRTUAL_MAGNETOMETER = true;
   public static final RigidBodyTransform DEFAULT_MULTISENSE_TO_IMU_TRANSFORM = new RigidBodyTransform(new Quaternion(0.0, Math.PI/2.0, 0.0), new Vector3D(-0.04, 0.0, 0.0777));

   private static final boolean ESTIMATE_ANGULAR_VELOCITY_BIAS = false;

   private final MicroStrainUDPPacketListener imuListener;

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final FramePoint3D headPositionEstimateFromRobotModel;

   private final boolean getRobotConfigurationDataFromNetwork;

   /*
    this field is not final but is still real-time safe because
    the value returned by the UDP listener is a concurrent copier
    buffer element.
   */
   private MicroStrainData microStrainData = new MicroStrainData();
   private final YoFrameVector3D imuAngularVelocity;
   private final YoFrameVector3D imuLinearAcceleration;
   private final YoFrameVector3D imuMagneticNorthVector;

   private final YoPoint3D initialHeadPosition;
   private final YoPoint3D initialHeadOrientationYPR;
   private final YoFrameVector3D initialMagneticFieldVector;

   private final FrameVector3D virtualMagnetometerMeasurement = new FrameVector3D();

   private final YoBoolean inErrorState;

   public ReferenceFrame getImuFrame()
   {
      return imuFrame;
   }

   private ReferenceFrame imuFrame;

   public MultisenseSLWithMicroStrainHeadPoseEstimator(double dt, RigidBodyTransform imuToHeadTransform, PriorityParameters imuListenerPriority,
                                                       long microStrainSerialNumber, boolean getRobotConfigurationDataFromNetwork) throws IOException
   {
      super(dt, imuToHeadTransform, ESTIMATE_ANGULAR_VELOCITY_BIAS);

      if (imuListenerPriority != null)
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
         RealtimeRos2Node realtimeRos2Node = ROS2Tools
               .createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "multisense_microstrain_head_pose_estimator");
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, ROS2Tools::generateDefaultTopicName, this::onNewDataMessage);
      }

      headPositionEstimateFromRobotModel = new FramePoint3D(ReferenceFrame.getWorldFrame());

      YoVariableRegistry registry = getRegistry();

      String prefix = "MicroStrainData";
      imuAngularVelocity = new YoFrameVector3D(prefix, "AngularVelocity", ReferenceFrame.getWorldFrame(), registry);
      imuLinearAcceleration = new YoFrameVector3D(prefix, "LinearAcceleration", ReferenceFrame.getWorldFrame(), registry);
      imuMagneticNorthVector = new YoFrameVector3D(prefix, "MagneticNorth", ReferenceFrame.getWorldFrame(), registry);

      initialHeadPosition = new YoPoint3D("EstimatedHeadPose", "InitialHeadPosition", registry);
      initialHeadOrientationYPR = new YoPoint3D(new YoDouble("EstimatedHeadPoseInitialYaw", registry), new YoDouble("EstimatedHeadPoseInitialPitch", registry), new YoDouble("EstimatedHeadPoseInitialRoll", registry));
      initialMagneticFieldVector = new YoFrameVector3D("EstimatedHeadPose", "InitialMagneticField", ReferenceFrame.getWorldFrame(), registry);

      inErrorState = new YoBoolean("inErrorState", registry);
      inErrorState.set(false);

   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      super.setFullRobotModel(fullRobotModel);

      if(USE_VIRTUAL_MAGNETOMETER)
      {
         imuFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("VirtualMagnetometerIMUFrame", fullRobotModel.getHead().getBodyFixedFrame(), DEFAULT_MULTISENSE_TO_IMU_TRANSFORM);
      }
   }

   @Override
   public void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection)
   {
      FrameVector3D initialMagneticFieldDirection = null;
      if (magneticFieldDirection == null)
      {
         if(!USE_VIRTUAL_MAGNETOMETER)
         {
            microStrainData = imuListener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);
            Tuple3DBasics geomagneticNorthVector = new Vector3D(microStrainData.getGeomagneticNorthVector());
            MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD.transform(geomagneticNorthVector);
            initialMagneticFieldDirection = new FrameVector3D(ReferenceFrame.getWorldFrame(), geomagneticNorthVector);
         }
         else
         {
            initialMagneticFieldDirection = new FrameVector3D(ReferenceFrame.getWorldFrame(), YoIMUMahonyFilter.NORTH_REFERENCE);
         }
      }
      else
      {
         initialMagneticFieldDirection = magneticFieldDirection;
      }

      this.initialHeadPosition.set(initialHeadTransform.getTranslation());
      this.initialHeadOrientationYPR.set(initialHeadTransform.getRotation().getYaw(), initialHeadTransform.getRotation().getPitch(), initialHeadTransform.getRotation().getRoll());
      this.initialMagneticFieldVector.set(initialMagneticFieldDirection);

      inErrorState.set(false);

      super.initialize(initialHeadTransform, initialMagneticFieldDirection);
   }

   @Override
   public void compute()
   {
      microStrainData = imuListener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);

      if (getRobotConfigurationDataFromNetwork)
      {
         boolean modelUpdatedWithNewData = robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(getFullRobotModel(), null);
         if (modelUpdatedWithNewData)
         {
            headPositionEstimateFromRobotModel.set(getFullRobotModel().getHead().getBodyFixedFrame().getTransformToWorldFrame().getTranslation());
            super.setEstimatedHeadPosition(headPositionEstimateFromRobotModel);
         }
      }
      else
      {
         headPositionEstimateFromRobotModel.set(getFullRobotModel().getHead().getBodyFixedFrame().getTransformToWorldFrame().getTranslation());
         super.setEstimatedHeadPosition(headPositionEstimateFromRobotModel);
      }

      if (microStrainData != null)
      {
         Vector3DReadOnly angularRate = microStrainData.getAngularRate();
         Vector3DReadOnly linearAcceleration = microStrainData.getLinearAcceleration();
         Vector3DReadOnly geomagneticNorthVector = null;

         if(!USE_VIRTUAL_MAGNETOMETER)
         {
           geomagneticNorthVector = microStrainData.getGeomagneticNorthVector();
           imuMagneticNorthVector.set(geomagneticNorthVector);
         }
         else
         {
            imuFrame.update();
            virtualMagnetometerMeasurement.setIncludingFrame(ReferenceFrame.getWorldFrame(), YoIMUMahonyFilter.NORTH_REFERENCE);
            virtualMagnetometerMeasurement.changeFrame(imuFrame);
            imuMagneticNorthVector.setMatchingFrame(virtualMagnetometerMeasurement);
         }

         imuAngularVelocity.set(angularRate);
         imuLinearAcceleration.set(linearAcceleration);

         imuLinearAcceleration.scale(MicroStrainData.MICROSTRAIN_GRAVITY);

         super.setImuAngularVelocity(imuAngularVelocity);
         super.setImuLinearAcceleration(imuLinearAcceleration);
         super.setImuMagneticFieldVector(virtualMagnetometerMeasurement);
      }

      try
      {
         super.compute();
      }
      catch(Throwable t)
      {
         if(!inErrorState.getBooleanValue())
         {
            System.err.println("Caught a throwable from the head pose EKF, ignoring.");
            inErrorState.set(true);
         }
      }

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
