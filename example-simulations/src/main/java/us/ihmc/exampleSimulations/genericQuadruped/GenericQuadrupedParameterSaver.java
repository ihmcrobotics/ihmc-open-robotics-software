package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.communication.remote.RemoteParameterSaver;
import us.ihmc.communication.util.NetworkPorts;

import java.io.IOException;
import java.nio.file.Paths;

public class GenericQuadrupedParameterSaver
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      RemoteParameterSaver.run(args, Paths.get("IHMCOpenRoboticsSoftware/exampleSimulations/src/main/resources/parameters"), NetworkPorts.CONTROLLER_PORT, new GenericQuadrupedNetClassList());
   }
}
