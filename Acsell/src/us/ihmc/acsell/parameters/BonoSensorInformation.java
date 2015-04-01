package us.ihmc.acsell.parameters;

import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class BonoSensorInformation implements DRCRobotSensorInformation
{
   
   public static final String lidarSensorName = null;
   public static final String leftCameraName = null;
   public static final String rightCameraName = null;
   public static final String imuSensor = "pelvis_pelvisIMU";
   public static final String[] imuSensorsToUse = {imuSensor};
   private final String[] forceSensorNames;
   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();
   
   public BonoSensorInformation()
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
      return null;
   }
}
