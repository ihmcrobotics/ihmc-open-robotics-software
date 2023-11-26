package us.ihmc.rdx.simulation.scs2;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;
import us.ihmc.scs2.sharedMemory.LinkedBufferProperties;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class RDXYoManager
{
   private YoRegistry rootRegistry;
   private LinkedYoRegistry linkedRootRegistry;
   private LinkedBufferProperties linkedBufferProperties;
   private LinkedYoVariableFactory linkedYoVariableFactory;
   private YoVariableDatabase rootRegistryDatabase = null;

   public RDXYoManager()
   {
   }

   public void startSession(Session session)
   {
      LogTools.info("Linking YoVariables");
      rootRegistry = session.getRootRegistry();
      linkedYoVariableFactory = session.getLinkedYoVariableFactory();
      linkedRootRegistry = linkedYoVariableFactory.newLinkedYoRegistry(rootRegistry);
      linkedBufferProperties = linkedYoVariableFactory.newLinkedBufferProperties();
      rootRegistryDatabase = new YoVariableDatabase(rootRegistry, linkedRootRegistry);
      LogTools.info("UI linked YoVariables created");
   }

   public void update()
   {
      linkedRootRegistry.pull();
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

   public LinkedBufferProperties newLinkedBufferProperties()
   {
      return linkedYoVariableFactory.newLinkedBufferProperties();
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   public LinkedYoRegistry getLinkedRootRegistry()
   {
      return linkedRootRegistry;
   }

   public LinkedYoVariableFactory getLinkedYoVariableFactory()
   {
      return linkedYoVariableFactory;
   }

   public LinkedBufferProperties getLinkedBufferProperties()
   {
      return linkedBufferProperties;
   }

   public int getBufferSize()
   {
      if (linkedBufferProperties == null || linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getSize();
   }

   public int getInPoint()
   {
      if (linkedBufferProperties == null || linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getInPoint();
   }

   public int getOutPoint()
   {
      if (linkedBufferProperties == null || linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getOutPoint();
   }

   public int getCurrentIndex()
   {
      if (linkedBufferProperties == null || linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getCurrentIndex();
   }

   public int getActiveBufferLength()
   {
      if (linkedBufferProperties == null || linkedBufferProperties.peekCurrentBufferProperties() == null)
         return -1;
      return linkedBufferProperties.peekCurrentBufferProperties().getActiveBufferLength();
   }
}
