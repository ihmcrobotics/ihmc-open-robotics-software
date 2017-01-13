package us.ihmc.graphics3DAdapter.jme;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.ContextManager;

/**
 * User: Matt
 * Date: 1/11/13
 */
public abstract class JMEContextManager implements ContextManager
{
   private JMERenderer jmeRenderer;

   private boolean enableSwitching = true;
   private ArrayList<InputMapSetter> inputMapSetters = new ArrayList<InputMapSetter>();

   protected ArrayList<JMEViewportAdapter> viewports = new ArrayList<JMEViewportAdapter>();
   private JMEViewportAdapter currentViewport;

   public JMEContextManager(JMERenderer jmeRenderer)
   {
      this.jmeRenderer = jmeRenderer;
   }

   public void setSwitchingEnabled(boolean enable)
   {
      enableSwitching = enable;
   }

   public void registerInputMapSetter(InputMapSetter inputMapSetter)
   {
      inputMapSetters.add(inputMapSetter);
   }

   public JMEViewportAdapter getCurrentViewport()
   {
      return currentViewport;
   }

   public boolean isSwitchingEnabled()
   {
      return enableSwitching;
   }

   public void addJMEViewportAdapter(JMEViewportAdapter jmeViewportAdapter)
   {
      viewports.add(jmeViewportAdapter);
      addJMEViewportAdapterToContext(jmeViewportAdapter);
   }

   public abstract void initialize();

   protected abstract void addJMEViewportAdapterToContext(JMEViewportAdapter jmeViewportAdapter);

   public abstract void focusOnCurrentWindow();

   protected void setCurrentViewport(JMEViewportAdapter currentViewport)
   {
      this.currentViewport = currentViewport;

      jmeRenderer.getInputManager().clearMappings();
      getCurrentViewport().setDefaultInputMappings();

      for (InputMapSetter inputMapSetter : inputMapSetters)
      {
         inputMapSetter.setDefaultInputMappings();
      }
   }

   protected void resetViewport(JMEViewportAdapter viewport)
   {
      if (viewport != null)
      {
         viewport.reset();
      }

      for (InputMapSetter inputMapSetter : inputMapSetters)
      {
         inputMapSetter.reset();
      }
   }

   public void closeAndDispose()
   {
      this.jmeRenderer = null;

      if (inputMapSetters != null)
      {
         inputMapSetters.clear();
         inputMapSetters = null;
      }

      if (viewports != null)
      {
         viewports.clear();
         viewports = null;
      }

      currentViewport = null;
   }
}
