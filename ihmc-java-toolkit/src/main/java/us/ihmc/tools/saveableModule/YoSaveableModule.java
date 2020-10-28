package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class YoSaveableModule<T extends YoSaveableModuleState>
{
   public T state;

   public YoSaveableModule(Class<? extends YoSaveableModule> moduleName, YoRegistry registry)
   {
      this(moduleName.getSimpleName(), registry);
   }

   public YoSaveableModule(String moduleName, YoRegistry registry)
   {
      YoBoolean saveStateFileTrigger = new YoBoolean(moduleName + "SaveStateFileTrigger", registry);
      saveStateFileTrigger.addListener((v) ->
                                       {
                                          if (saveStateFileTrigger.getBooleanValue())
                                          {
                                             saveStateFileTrigger.set(false, false);
                                             try
                                             {
                                                YoSaveableModuleStateTools.save(moduleName, state);
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
