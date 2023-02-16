package us.ihmc.avatar.testTools;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;

import java.io.File;

public class BasicYovariableExport
{
   private final SimulationConstructionSet2 scs2;

   // SCS object to call

   // assume scs2 has been cropped already
   public BasicYovariableExport(SimulationConstructionSet2 scs2) // and list of yovariables
   {
      this.scs2 = scs2;
   }

   public void addYoVariableNameToExport(String name)
   {
      scs2.addSessionDataFilterParameters("Joint torques", v -> v.getName().startsWith("raw_tau_"));
   }

   // and list of yovariables to export
   public void export(File file)
   {
      SessionDataExportRequest request = new SessionDataExportRequest();
      request.setFile(file);
      request.setExportRobotDefinitions(false);
      request.setExportTerrainObjectDefinitions(false);
      request.setExportSessionYoGraphicDefinitions(false);
      request.setExportRobotStateDefinitions(false);

      request.setExportSessionBufferDataFormat(SharedMemoryIOTools.DataFormat.CSV);
      request.setExportSessionBufferRegistryDefinition(true);

      scs2.exportData(request);
   }
}
