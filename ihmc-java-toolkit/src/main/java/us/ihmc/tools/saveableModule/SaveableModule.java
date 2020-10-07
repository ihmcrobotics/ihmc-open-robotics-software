package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class SaveableModule<T extends SaveableModuleState>
{
   public T state;

   public SaveableModule(Class<? extends SaveableModule> moduleName, YoRegistry registry)
   {
      this(moduleName.getSimpleName(), registry);
   }

   public SaveableModule(String moduleName, YoRegistry registry)
   {
      YoBoolean saveStateFileTrigger = new YoBoolean(moduleName + "SaveStateFileTrigger", registry);
      saveStateFileTrigger.addListener((v) ->
                                       {
                                          if (saveStateFileTrigger.getBooleanValue())
                                          {
                                             saveStateFileTrigger.set(false, false);
                                             try
                                             {
                                                SaveableModuleStateTools.save(moduleName, state);
                                             }
                                             catch (Exception e)
                                             {

                                             }
                                          }
                                       });
   }

   public void registerState(T state)
   {
      this.state = state;
   }

   public abstract void compute(T state);
}
