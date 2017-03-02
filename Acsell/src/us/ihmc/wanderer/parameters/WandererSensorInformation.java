package us.ihmc.wanderer.parameters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

public class WandererSensorInformation implements DRCRobotSensorInformation
{

   public static final String lidarSensorName = null;
   public static final String leftCameraName = null;
   public static final String rightCameraName = null;
   public static final String imuSensor = "pelvis_pelvisIMU";
   public static final String[] imuSensorsToUse = {imuSensor};
   private final String[] forceSensorNames;
   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();

   public WandererSensorInformation()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         String robotSideLowerCaseFirstLetter = robotSide.getSideNameFirstLetter().toLowerCase();
         feetForceSensorNames.put(robotSide, robotSideLowerCaseFirstLetter + "_leg_lax");
      }
      forceSensorNames= new String[]{feetForceSensorNames.get(RobotSide.LEFT), feetForceSensorNames.get(RobotSide.RIGHT)};
   }

   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse;
   }



   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return new SideDependentList<String>(null, null);
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return imuSensor;
   }

   @Override
   public DRCRobotCameraParameters[] getCameraParameters()
   {
      return new DRCRobotCameraParameters[0];
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters(int sensorId)
   {
      return null;
   }

   @Override
   public DRCRobotLidarParameters[] getLidarParameters()
   {
      return new DRCRobotLidarParameters[0];
   }

   @Override
   public DRCRobotLidarParameters getLidarParameters(int sensorId)
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParameters[] getPointCloudParameters()
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return null;
   }

   @Override
   public String[] getSensorFramesToTrack()
   {
      return null;
   }

   @Override
   public boolean setupROSLocationService()
   {
      return false;
   }

   @Override
   public boolean setupROSParameterSetters()
   {
      return false;
   }

   @Override
   public boolean isMultisenseHead()
   {
      return false;
   }

   @Override
   public ReferenceFrame getHeadIMUFrameWhenLevel(){
	   return null;
   }

   @Override
   public SideDependentList<String> getFeetContactSensorNames()
   {
      return new SideDependentList<String>();
   }

   @Override
   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return null;
   }
}
