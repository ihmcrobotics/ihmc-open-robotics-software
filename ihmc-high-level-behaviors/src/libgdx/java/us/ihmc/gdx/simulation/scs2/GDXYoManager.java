package us.ihmc.gdx.simulation.scs2;

import javafx.beans.property.LongProperty;
import javafx.beans.property.SimpleLongProperty;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sharedMemory.LinkedBufferProperties;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class GDXYoManager
{
   private final LongProperty rootRegistryHashCodeProperty = new SimpleLongProperty(this, "rootRegistryHashCode", 0);

   private YoRegistry rootRegistry;
   private LinkedYoRegistry linkedRootRegistry;
   private LinkedBufferProperties linkedBufferProperties;
   private LinkedYoVariableFactory linkedYoVariableFactory;

   private boolean updatingYoVariables = true;

   public GDXYoManager()
   {
   }

   public void update()
   {
      if (linkedRootRegistry != null && !updatingYoVariables)
         linkedRootRegistry.pull();
   }

   public void startSession(Session session)
   {
      LogTools.info("Linking YoVariables");
      rootRegistry = new YoRegistry(SimulationSession.ROOT_REGISTRY_NAME);
      linkedYoVariableFactory = session.getLinkedYoVariableFactory();
      linkedRootRegistry = linkedYoVariableFactory.newLinkedYoRegistry(rootRegistry);
      linkedBufferProperties = linkedYoVariableFactory.newLinkedBufferProperties();

      LogTools.info("UI linked YoVariables created");
   }

   public void stopSession()
   {
      rootRegistry = null;
      linkedYoVariableFactory = null;
      linkedRootRegistry = null;
      linkedBufferProperties = null;
      rootRegistryHashCodeProperty.set(-1L);
   }

   public boolean isSessionLoaded()
   {
      return linkedRootRegistry != null && !updatingYoVariables;
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

   public boolean isUpdatingYoVariables()
   {
      return updatingYoVariables;
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

   public LongProperty rootRegistryHashCodeProperty()
   {
      return rootRegistryHashCodeProperty;
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
