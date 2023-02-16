package us.ihmc.avatar.testTools;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;

import java.io.File;
import java.util.ArrayList;

public class BasicYovariableExport
{
   private SimulationConstructionSet2 scs2;
   private final ArrayList<String> variableNamesToExport = new ArrayList<>();

   // assume scs2 has been cropped already
   public BasicYovariableExport() // and list of yovariables
   {

   }

   public void setSCSToExport(SimulationConstructionSet2 scs2)
   {
      this.scs2 = scs2;
   }

   public void addYoVariableNameToExport(String name)
   {
      variableNamesToExport.add(name);
   }

   public void export(File file)
   {
      SessionDataExportRequest request = new SessionDataExportRequest();
      request.setVariableFilter(v -> variableNamesToExport.contains(v.getName()));
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
