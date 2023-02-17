package us.ihmc.avatar.testTools;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryIOTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class BasicYovariableExport
{
   private SimulationConstructionSet2 scs2;
   private final ArrayList<String> individualVariableNamesToExport = new ArrayList<>();
   private final List<String> registryNamesToExport = new ArrayList<>();

   private final List<String> registryVariableNamesToExport = new ArrayList<>();
   private YoRegistry parentRegistry;

   // assume scs2 has been cropped already
   public BasicYovariableExport() // and list of yovariables
   {
   }

   // Must be called first
   public void setSCSToExport(SimulationConstructionSet2 scs2)
   {
      this.scs2 = scs2;
      this.parentRegistry = scs2.getRootRegistry();
   }

   public void addYoVariableNameToExport(String name)
   {
      individualVariableNamesToExport.add(name);
   }

   public void addYoRegistryNameToExport(String registryName)
   {
      if (!registryNamesToExport.contains(registryName))
      {
         registryNamesToExport.add(registryName);
      }
   }

   private void compileRegistryYoVariableNames()
   {
      registryVariableNamesToExport.clear();
      for (String registryName : registryNamesToExport)
      {
         System.out.println("Exporting: " + registryName);
         YoRegistry registry = parentRegistry.findRegistry(new YoNamespace(registryName));
         List<YoVariable> yoList = registry.collectSubtreeVariables();
         for (YoVariable var : yoList)
         {
            if (!registryVariableNamesToExport.contains(var.getName()))
               registryVariableNamesToExport.add(var.getFullNameString());
         }
      }
   }

   public void export(File file)
   {
      SessionDataExportRequest request = new SessionDataExportRequest();
      compileRegistryYoVariableNames();
      request.setVariableFilter(v -> (registryVariableNamesToExport.contains(v.getFullNameString()) ||
                                      individualVariableNamesToExport.contains(v.getName()))
                              );
//      request.setRegistryFilter(r -> {
////         return ((registryNamesToExport.contains(r.getName())) &&
////          (parentRegistry.getRoot().findRegistry(new YoNamespace(r.getName())) != null));
//      });
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
