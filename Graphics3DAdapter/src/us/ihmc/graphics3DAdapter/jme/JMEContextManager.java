package us.ihmc.graphics3DAdapter.jme;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.ContextManager;

import com.jme3.math.ColorRGBA;

/**
 * User: Matt
 * Date: 1/11/13
 */
public abstract class JMEContextManager implements ContextManager
{
   private final JMERenderer jmeRenderer;
   
   private boolean enableSwitching = true;
   private ArrayList<InputMapSetter> inputMapSetters = new ArrayList<InputMapSetter>();
   
   protected final ArrayList<JMEViewportAdapter> viewports = new ArrayList<JMEViewportAdapter>();
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
      
      for (JMEViewportAdapter viewport : viewports)
      {
         if (viewport == currentViewport)
         {
            viewport.getViewPort().setBackgroundColor(new ColorRGBA(0.7f, 0.8f, 1f, 1f));
         }
         else
         {
            viewport.getViewPort().setBackgroundColor(new ColorRGBA(0.7f, 0.8f, 0.8f, 1f));
         }
      }
   }
   
   protected void resetViewport(JMEViewportAdapter viewport)
   {
      if(viewport != null)
      {
         viewport.reset();
      }

      for (InputMapSetter inputMapSetter : inputMapSetters)
      {
         inputMapSetter.reset();
      }
   }
}
