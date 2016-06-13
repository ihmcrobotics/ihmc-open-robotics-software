package us.ihmc.llaQuadruped;

import java.io.IOException;
import java.nio.file.Paths;

import us.ihmc.quadrupedRobotics.params.ParameterSaver;

public class LLAQuadrupedParameterSaver
{
   public static void main(String[] args) throws IOException
   {
      ParameterSaver.run(args, Paths.get("IHMCOpenRoboticsSoftware/LLAQuadruped/resources/parameters"), new LLAQuadrupedNetClassList());
   }
}
