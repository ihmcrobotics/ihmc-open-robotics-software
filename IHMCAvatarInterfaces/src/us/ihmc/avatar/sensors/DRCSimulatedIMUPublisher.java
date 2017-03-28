package us.ihmc.avatar.sensors;

import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.sensing.RawIMUPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;

public class DRCSimulatedIMUPublisher implements MultiThreadedRobotControlElement
{

   HumanoidGlobalDataProducer globalDataProducer;
   RawIMUPacket packet = new RawIMUPacket();
   IMUSensorReadOnly imuSensorReader = null;

   public DRCSimulatedIMUPublisher(HumanoidGlobalDataProducer globalDataProducer, List<? extends IMUSensorReadOnly> simulatedIMUOutput, String headLinkName)
   {

      for (int i = 0; i < simulatedIMUOutput.size(); i++)
      {
         if (simulatedIMUOutput.get(i).getSensorName().contains(headLinkName))
         {
            imuSensorReader = simulatedIMUOutput.get(i);
            break;
         }
      }
      this.globalDataProducer = globalDataProducer;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void read(long currentClockTime)
   {
      if (imuSensorReader != null)
      {
         imuSensorReader.getLinearAccelerationMeasurement(packet.linearAcceleration);
         packet.timestampInNanoSecond = currentClockTime;
         if (globalDataProducer != null)
            globalDataProducer.queueDataToSend(packet);
      }
   }

   @Override
   public void run()
   {

   }

   @Override
   public void write(long timestamp)
   {
      //not writing to robot
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public String getName()
   {
      return "SlowPublisher";
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }

   @Override
   public long nextWakeupTime()
   {
      return 0;
   }

}
