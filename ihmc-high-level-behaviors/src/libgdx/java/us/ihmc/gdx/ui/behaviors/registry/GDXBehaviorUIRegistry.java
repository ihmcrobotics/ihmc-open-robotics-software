package us.ihmc.gdx.ui.behaviors.registry;

import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;

import java.util.LinkedHashSet;

public class GDXBehaviorUIRegistry extends BehaviorRegistry
{
   public static final GDXBehaviorUIRegistry DEFAULT_BEHAVIORS = new GDXBehaviorUIRegistry();
   static
   {
      DEFAULT_BEHAVIORS.register(LookAndStepBehavior.DEFINITION);
   }

   private final LinkedHashSet<GDXBehaviorUIDefinition> uiDefinitionEntries = new LinkedHashSet<>();
   private int numberOfUIs = 0;
   private String nameOfOnlyUIBehavior;

   public static GDXBehaviorUIRegistry of(GDXBehaviorUIDefinition... entries)
   {
      GDXBehaviorUIRegistry registry = new GDXBehaviorUIRegistry();
      for (GDXBehaviorUIDefinition entry : entries)
      {
         registry.register(entry);
      }
      return registry;
   }

   public void register(GDXBehaviorUIDefinition definition)
   {
      super.register(definition);
      uiDefinitionEntries.add(definition);

      if (definition.getBehaviorUISupplier() != null)
      {
         ++numberOfUIs;
         nameOfOnlyUIBehavior = definition.getName();
      }
   }

   public LinkedHashSet<GDXBehaviorUIDefinition> getUIDefinitionEntries()
   {
      return uiDefinitionEntries;
   }

   public int getNumberOfUIs()
   {
      return numberOfUIs;
   }

   public String getNameOfOnlyUIBehavior()
   {
      return nameOfOnlyUIBehavior;
   }
}
