package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packets.Packet;

@ClassDocumentation("This message will request a calibration the wrist force sensors to the IHMC controller (does not do BDI calibration).\n"
		                            + "It is strongly suggested to perform the calibration when the hands are not moving nor interacting with the environment.")
public class RequestWristForceSensorCalibrationPacket extends Packet<RequestWristForceSensorCalibrationPacket>
{
   public RequestWristForceSensorCalibrationPacket()
   {
   }

   @Override
   public boolean epsilonEquals(RequestWristForceSensorCalibrationPacket other, double epsilon)
   {
      return true;
   }

}
