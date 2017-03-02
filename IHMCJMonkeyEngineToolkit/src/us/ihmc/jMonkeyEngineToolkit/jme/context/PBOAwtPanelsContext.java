package us.ihmc.jMonkeyEngineToolkit.jme.context;

import java.util.ArrayList;

import com.jme3.input.JoyInput;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.TouchInput;
import com.jme3.input.awt.AwtKeyInput;
import com.jme3.input.awt.AwtMouseInput;
import com.jme3.renderer.Renderer;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext;
import com.jme3.system.JmeSystem;
import com.jme3.system.SystemListener;
import com.jme3.system.Timer;

public class PBOAwtPanelsContext implements JmeContext
{
   private static final boolean DEBUG = false;
   
   private JmeContext actualContext;
   private AppSettings settings = new AppSettings(true);
   private SystemListener listener;
   private ArrayList<PBOAwtPanel> panels = new ArrayList<>();
   private PBOAwtPanel inputSource;

   private AwtMouseInput mouseInput = new AwtMouseInput();
   private AwtKeyInput keyInput = new AwtKeyInput();

   private boolean lastThrottleState = false;

   private ArrayList<PBOAwtPanelListener> pboAwtPanelListeners = new ArrayList<>();

   private class AwtPanelsListener implements SystemListener
   {
      public void initialize()
      {
         initInThread();
      }

      public void reshape(int width, int height)
      {
         throw new IllegalStateException();
      }

      public void update()
      {
         updateInThread();
      }

      public void requestClose(boolean esc)
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      public void gainFocus()
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      public void loseFocus()
      {
         // shouldn't happen
         throw new IllegalStateException();
      }

      public void handleError(String errorMsg, Throwable t)
      {
         listener.handleError(errorMsg, t);
      }

      public void destroy()
      {
         destroyInThread();
      }
   }


   public void addPBOAwtPanelListener(PBOAwtPanelListener listener)
   {
      pboAwtPanelListeners.add(listener);
   }

   public void setInputSource(PBOAwtPanel panel)
   {
      if (!panels.contains(panel))
         throw new IllegalArgumentException();

      inputSource = panel;
      mouseInput.setInputSource(panel);
      keyInput.setInputSource(panel);
   }

   public ArrayList<PBOAwtPanel> getPanelList()
   {
      return panels;
   }

   public Type getType()
   {
      return Type.OffscreenSurface;
   }

   public void setSystemListener(SystemListener listener)
   {
      this.listener = listener;
   }

   public AppSettings getSettings()
   {
      return settings;
   }

   public Renderer getRenderer()
   {
      return actualContext.getRenderer();
   }

   public MouseInput getMouseInput()
   {
      return mouseInput;
   }

   public KeyInput getKeyInput()
   {
      return keyInput;
   }

   public JoyInput getJoyInput()
   {
      return null;
   }

   public TouchInput getTouchInput()
   {
      return null;
   }

   public Timer getTimer()
   {
      return actualContext.getTimer();
   }

   public boolean isCreated()
   {
      return (actualContext != null) && actualContext.isCreated();
   }

   public boolean isRenderable()
   {
      return (actualContext != null) && actualContext.isRenderable();
   }

   public PBOAwtPanelsContext()
   {
   }

   public PBOAwtPanel createPanel()
   {
      if (alreadyDestroying) return null;

      PBOAwtPanel panel = new PBOAwtPanel(pboAwtPanelListeners);
      panels.add(panel);

      for (PBOAwtPanelListener listener : pboAwtPanelListeners)
      {
         listener.isCreated(panel);
      }

      return panel;
   }

   private void initInThread()
   {
      listener.initialize();
   }

   private void updateInThread()
   {
      if (alreadyDestroying) return;
      
      // Check if throttle required
      boolean needThrottle = true;

      for (PBOAwtPanel panel : panels)
      {
         if (panel.isActiveDrawing())
         {
            needThrottle = false;

            break;
         }
      }

      if (lastThrottleState != needThrottle)
      {
         lastThrottleState = needThrottle;

         if (lastThrottleState)
         {
        	 printIfDebug(getClass().getSimpleName() + ": Throttling update loop.");
         }
         else
         {
        	 printIfDebug(getClass().getSimpleName() + ": Ceased throttling update loop.");
         }
      }

      if (needThrottle)
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

      if (!alreadyDestroying) listener.update();
   }

boolean alreadyDestroying = false;
   
   private void destroyInThread()
   {
      if (alreadyDestroying) return;
      alreadyDestroying = true;
      
      listener.destroy();

      if (pboAwtPanelListeners != null)
      {
         pboAwtPanelListeners.clear();
         pboAwtPanelListeners = null;
      }

      if (panels != null)
      {
         for (PBOAwtPanel pboAwtPanel : panels)
         {
            pboAwtPanel.closeAndDispose();
         }
         
         panels.clear();
      }

      actualContext = null;
      settings = null;
      inputSource = null;

      mouseInput = null;
      keyInput = null;
   }

   public void setSettings(AppSettings settings)
   {
      this.settings.copyFrom(settings);
      this.settings.setRenderer(AppSettings.LWJGL_OPENGL2);

      if (actualContext != null)
      {
         actualContext.setSettings(settings);
      }
   }

   public void create(boolean waitFor)
   {
      if (actualContext != null)
      {
         throw new IllegalStateException("Already created");
      }

      actualContext = JmeSystem.newContext(settings, Type.OffscreenSurface);
      actualContext.setSystemListener(new AwtPanelsListener());
      actualContext.create(waitFor);
   }

   public void destroy(boolean waitFor)
   {
      if (actualContext == null)
         throw new IllegalStateException("Not created");

      // destroy parent context
      actualContext.destroy(waitFor);
   }

   public void setTitle(String title)
   {
      // not relevant, ignore
   }

   public void setAutoFlushFrames(boolean enabled)
   {
      // not relevant, ignore
   }

   public void restart()
   {
      // only relevant if changing pixel format.
   }
   
   private void printIfDebug(String string) 
   {
	   if (DEBUG) System.out.println(string);
   }

}
