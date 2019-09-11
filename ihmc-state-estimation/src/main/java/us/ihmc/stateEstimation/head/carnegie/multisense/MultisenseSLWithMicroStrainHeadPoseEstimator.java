package us.ihmc.stateEstimation.head.carnegie.multisense;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainData;
import us.ihmc.sensors.imu.lord.microstrain.MicroStrainUDPPacketListener;
import us.ihmc.stateEstimation.head.HeadPoseEstimator;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultisenseSLWithMicroStrainHeadPoseEstimator
{
   private static final String PARAMETER_FILE = "headPoseEstimatorTest.xml";
   private static final boolean ESTIMATE_ANGULAR_VELOCITY_BIAS = true;

   private final String simpleName = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(simpleName);

   private final HeadPoseEstimator headPoseEstimator;
   private final MicroStrainUDPPacketListener imuListener;

   private final RigidBodyTransform estimatedHeadTransform = new RigidBodyTransform();

   /*
    this is not final but is still real-time safe because
    the value returned by the UDP listener is a concurrent copier
    buffer element.
   */
   private MicroStrainData microStrainData = new MicroStrainData();

   public MultisenseSLWithMicroStrainHeadPoseEstimator(double dt, RigidBodyTransform imuToHeadTransform, PriorityParameters imuListenerPriority, long microStrainSerialNumber, YoVariableRegistry parentRegistry)
         throws IOException
   {
      headPoseEstimator = new HeadPoseEstimator(dt, imuToHeadTransform, ESTIMATE_ANGULAR_VELOCITY_BIAS, registry);

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

      parentRegistry.addChild(registry);
   }

   public void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection)
   {
      headPoseEstimator.initialize(initialHeadTransform, magneticFieldDirection);
   }

   public void update()
   {
      microStrainData = imuListener.getLatestData(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER);

      if(microStrainData != null)
      {
         headPoseEstimator.setImuAngularVelocity(microStrainData.getAngularRate());
         headPoseEstimator.setImuLinearAcceleration(microStrainData.getLinearAcceleration());
         headPoseEstimator.setImuMagneticFieldVector(microStrainData.getGeomagneticNorthVector());
         headPoseEstimator.setEstimatedHeadPosition(null); //TODO this
         headPoseEstimator.compute();
      }

      headPoseEstimator.getHeadTransform(estimatedHeadTransform);
   }
}
