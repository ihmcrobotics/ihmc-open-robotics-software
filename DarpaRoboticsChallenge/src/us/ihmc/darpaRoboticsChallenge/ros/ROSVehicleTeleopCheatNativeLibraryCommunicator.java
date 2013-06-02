package us.ihmc.darpaRoboticsChallenge.ros;

public class ROSVehicleTeleopCheatNativeLibraryCommunicator
{
   private static ROSVehicleTeleopCheatNativeLibraryCommunicator instance = null;
   private static String rosMasterURI;

   private ROSVehicleTeleopCheatNativeLibraryCommunicator(String rosMasterURI)
   {
   }

   public static ROSVehicleTeleopCheatNativeLibraryCommunicator getInstance(String rosMasterURI)
   {
      if (instance == null)
      {
         instance = new ROSVehicleTeleopCheatNativeLibraryCommunicator(rosMasterURI);
      }
      else if (!rosMasterURI.equals(""))
      {
         throw new RuntimeException("Cannot get an instance of ROSVehicleTeleopCheatNativeLibraryCommunicator for " + rosMasterURI + ", already connected to "
                                    + ROSVehicleTeleopCheatNativeLibraryCommunicator.rosMasterURI);
      }

      return instance;
   }
}
