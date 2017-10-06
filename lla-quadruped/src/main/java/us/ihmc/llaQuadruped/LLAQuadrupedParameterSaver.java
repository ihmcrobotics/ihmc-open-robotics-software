package us.ihmc.llaQuadruped;

import java.io.IOException;
import java.nio.file.Paths;

import us.ihmc.communication.remote.RemoteParameterSaver;
import us.ihmc.communication.util.NetworkPorts;

public class LLAQuadrupedParameterSaver
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      RemoteParameterSaver.run(args, Paths.get("IHMCOpenRoboticsSoftware/LLAQuadruped/resources/parameters"), NetworkPorts.CONTROLLER_PORT, new LLAQuadrupedNetClassList());
   }
}
