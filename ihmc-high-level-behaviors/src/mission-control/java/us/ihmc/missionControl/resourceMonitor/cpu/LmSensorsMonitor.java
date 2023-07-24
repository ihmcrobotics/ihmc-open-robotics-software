package us.ihmc.missionControl.resourceMonitor.cpu;

import us.ihmc.log.LogTools;
import us.ihmc.missionControl.resourceMonitor.ResourceMonitor;

import java.util.HashMap;
import java.util.Map;

// Requires lm_sensors
public class LmSensorsMonitor extends ResourceMonitor
{
   private final Map<Integer, Integer> cpuTemps = new HashMap<>();

   public LmSensorsMonitor()
   {
      super("sensors");
   }

   public Map<Integer, Integer> getCpuTemps()
   {
      return cpuTemps;
   }

   /*
      Example sensors output

      iwlwifi_1-virtual-0
      Adapter: Virtual device
      temp1:            N/A

      pch_skylake-virtual-0
      Adapter: Virtual device
      temp1:        +39.0°C

      BAT0-acpi-0
      Adapter: ACPI interface
      in0:          11.88 V

      coretemp-isa-0000
      Adapter: ISA adapter
      Package id 0:  +39.0°C  (high = +100.0°C, crit = +100.0°C)
      Core 0:        +39.0°C  (high = +100.0°C, crit = +100.0°C)
      Core 1:        +38.0°C  (high = +100.0°C, crit = +100.0°C)
    */

   @Override
   public void parse(String[] lines)
   {
      for (int i = 0; i < lines.length; i++)
      {
         if (lines[i].contains("coretemp-isa-0000"))
         {
            int nextCPULine = i + 3;
            try
            {
               while (lines[nextCPULine].trim().startsWith("Core"))
               {
                  int cpu = Integer.parseInt(lines[nextCPULine].split("\\s+")[1].replace(":", ""));
                  String tempString = lines[nextCPULine].split("\\s+")[2];
                  int temp = (int) Float.parseFloat(tempString.replace("\u00B0C", "")); // \u00B0 is the degrees symbol
                  cpuTemps.put(cpu, temp);
                  nextCPULine++;
               }
            }
            catch (ArrayIndexOutOfBoundsException | NumberFormatException ignored)
            {
               LogTools.info("Unable to parse lm_sensors");
               return; // stopgap
            }
         }
      }
   }
}
