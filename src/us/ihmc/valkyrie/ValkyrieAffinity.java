package us.ihmc.valkyrie;

import java.util.logging.Logger;

import us.ihmc.realtime.affinity.CPUTopology;
import us.ihmc.realtime.affinity.Processor;

public class ValkyrieAffinity
{
   private final static Logger log = Logger.getLogger(ValkyrieAffinity.class.getName());
   
   private final boolean setAffinity;
  
   private final Processor estimatorThreadProcessor;
   private final Processor controlThreadProcessor;
   
   public ValkyrieAffinity()
   {
      CPUTopology topology = new CPUTopology();
      
      if(topology.isHyperThreadingEnabled())
      {
         log.severe("WARNING: Hyper-Threading is enabled. Expect higher amounts of jitter");
      }
      
      if(topology.getNumberOfCores() < 8)
      {
         log.config("Number of cores < 8. Disabling affinity");
         setAffinity = false;
         estimatorThreadProcessor = null;
         controlThreadProcessor = null;
      }
      else
      {
         log.config("Number of cores >= 8. Pinning control threads to processor 1 & 2.");
         setAffinity = true;
         us.ihmc.realtime.affinity.Package socket = topology.getPackage(0);
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
