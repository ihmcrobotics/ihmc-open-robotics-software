package us.ihmc.valkyrie;

import us.ihmc.affinity.CPUTopology;
import us.ihmc.affinity.Package;
import us.ihmc.affinity.Processor;

public class ValkyrieAffinity
{
   private final boolean setAffinity;
  
   private final Processor estimatorThreadProcessor;
   private final Processor controlThreadProcessor;
   
   public ValkyrieAffinity()
   {
      CPUTopology topology = new CPUTopology();
      
      if(topology.isHyperThreadingEnabled())
      {
         System.err.println("WARNING: Hyper-Threading is enabled. Expect higher amounts of jitter");
      }
      
      if(topology.getNumberOfCores() < 8)
      {
         System.out.println("Number of cores < 8. Disabling affinity");
         setAffinity = false;
         estimatorThreadProcessor = null;
         controlThreadProcessor = null;
      }
      else
      {
         System.out.println("Number of cores >= 8. Pinning control threads to processor 1 & 2.");
         setAffinity = true;
         Package socket = topology.getPackage(0);
         estimatorThreadProcessor = socket.getCore(1).getDefaultProcessor();
         controlThreadProcessor = socket.getCore(2).getDefaultProcessor();
      }
      

   }

   public Processor getEstimatorThreadProcessor()
   {
      if(!setAffinity)
      {
         throw new RuntimeException("Setting affinity is disabled");
      }
      return estimatorThreadProcessor;
   }

   public Processor getControlThreadProcessor()
   {
      if(!setAffinity)
      {
         throw new RuntimeException("Setting affinity is disabled");
      }
      return controlThreadProcessor;
   }

   public boolean setAffinity()
   {
      return setAffinity;
   }
   
   
}
