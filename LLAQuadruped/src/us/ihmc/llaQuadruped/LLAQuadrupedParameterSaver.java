package us.ihmc.llaQuadruped;

import java.io.IOException;
import java.nio.file.Paths;

import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.communication.remote.ParameterSaver;

public class LLAQuadrupedParameterSaver
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      ParameterSaver.run(args, Paths.get("IHMCOpenRoboticsSoftware/LLAQuadruped/resources/parameters"), NetworkPorts.CONTROLLER_PORT, new LLAQuadrupedNetClassList());
   }
}
