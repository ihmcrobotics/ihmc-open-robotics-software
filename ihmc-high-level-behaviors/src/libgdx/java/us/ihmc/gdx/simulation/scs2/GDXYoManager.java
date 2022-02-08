package us.ihmc.gdx.simulation.scs2;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sharedMemory.LinkedBufferProperties;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class GDXYoManager
{
   private YoRegistry rootRegistry;
   private LinkedYoRegistry linkedRootRegistry;
   private LinkedBufferProperties linkedBufferProperties;
   private LinkedYoVariableFactory linkedYoVariableFactory;
   private YoVariableDatabase rootRegistryDatabase = null;

   public GDXYoManager()
   {
   }

   public void startSession(Session session)
   {
      LogTools.info("Linking YoVariables");
      rootRegistry = new YoRegistry(SimulationSession.ROOT_REGISTRY_NAME);
      linkedYoVariableFactory = session.getLinkedYoVariableFactory();
      linkedRootRegistry = linkedYoVariableFactory.newLinkedYoRegistry(rootRegistry);
      linkedBufferProperties = linkedYoVariableFactory.newLinkedBufferProperties();
      rootRegistryDatabase = new YoVariableDatabase(rootRegistry, linkedRootRegistry);
      LogTools.info("UI linked YoVariables created");
   }

   public void update()
   {
      if (linkedRootRegistry != null)
         linkedRootRegistry.pull();
   }

   public void stopSession()
   {
      rootRegistry = null;
      linkedYoVariableFactory = null;
      linkedRootRegistry = null;
      linkedBufferProperties = null;
   }

   public boolean isSessionLoaded()
   {
      return linkedRootRegistry != null;
   }

   public LinkedYoRegistry newLinkedYoRegistry(YoRegistry registry)
   {
      return linkedYoVariableFactory.newLinkedYoRegistry(registry);
   }

   public LinkedYoVariable<?> newLinkedYoVariable(YoVariable yoVariable)
   {
      return linkedYoVariableFactory.newLinkedYoVariable(yoVariable);
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   public LinkedYoRegistry getLinkedRootRegistry()
   {
      return linkedRootRegistry;
   }

   public LinkedBufferProperties newLinkedBufferProperties()
   {
      if (linkedYoVariableFactory == null)
         return null;
      else
         return linkedYoVariableFactory.newLinkedBufferProperties();
   }

   public int getBufferSize()
   {
      if (linkedBufferProperties == null)
         return -1;
      if (linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getSize();
   }
}
