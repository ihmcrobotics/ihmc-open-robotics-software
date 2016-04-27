package us.ihmc.quadrupedRobotics.input.devices;

public class NoInputDeviceException extends Exception
{
   public NoInputDeviceException(String deviceName)
   {
      super("No input device available: " + deviceName);
   }
}
