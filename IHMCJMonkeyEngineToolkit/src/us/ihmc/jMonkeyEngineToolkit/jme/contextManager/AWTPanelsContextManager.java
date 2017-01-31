package us.ihmc.jMonkeyEngineToolkit.jme.contextManager;

import java.awt.Canvas;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.LinkedHashMap;

import com.jme3.system.JmeContext;
import com.jme3.system.awt.AwtPanel;
import com.jme3.system.awt.AwtPanelsContext;

import us.ihmc.jMonkeyEngineToolkit.jme.JMEContextManager;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanel;
import us.ihmc.jMonkeyEngineToolkit.jme.context.PBOAwtPanelsContext;

public class AWTPanelsContextManager extends JMEContextManager implements MouseListener
{
   private JMERenderer jmeRenderer;
   private LinkedHashMap<Canvas, JMEViewportAdapter> panelViewports = new LinkedHashMap<Canvas, JMEViewportAdapter>();
   
   public AWTPanelsContextManager(JMERenderer jmeRenderer)
   {
      super(jmeRenderer);
      this.jmeRenderer = jmeRenderer;
   }

   @Override
   public void addJMEViewportAdapterToContext(JMEViewportAdapter jmeViewportAdapter)
   {
      panelViewports.put(jmeViewportAdapter.getCanvas(), jmeViewportAdapter);
      jmeViewportAdapter.getCanvas().addMouseListener(this);
   }
   
   
   public void mouseClicked(MouseEvent e)
   {
   }

   public void mousePressed(MouseEvent e)
   {
   }

   public void mouseReleased(MouseEvent e)
   {
   }

   public void mouseEntered(MouseEvent e)
   {
      if (isSwitchingEnabled())
      {
         if (jmeRenderer != null)
         {
            Canvas selectedAwtPanel;

            if (e != null)
               selectedAwtPanel = (Canvas) e.getComponent();
            else
               selectedAwtPanel = getCurrentViewport().getCanvas();

            if (selectedAwtPanel != null)
            {
               JmeContext context = jmeRenderer.getContext();
               if (context instanceof AwtPanelsContext)
               {
                  ((AwtPanelsContext) context).setInputSource((AwtPanel) selectedAwtPanel);
               }
               else if (context instanceof PBOAwtPanelsContext)
               {
                  ((PBOAwtPanelsContext) context).setInputSource((PBOAwtPanel) selectedAwtPanel);
               }

               setCurrentViewport(panelViewports.get(selectedAwtPanel));

               selectedAwtPanel.requestFocus();
            }
         }
      }
   }

   public void mouseExited(MouseEvent e)
   {
      if (isSwitchingEnabled())
      {
         Canvas exitedPanel = (Canvas) e.getComponent();
         JMEViewportAdapter exitedAdapter = panelViewports.get(exitedPanel);
         
         resetViewport(exitedAdapter);
      }
   }

   @Override
   public void focusOnCurrentWindow()
   {
       getCurrentViewport().getCanvas().requestFocus();
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }
   
   public void closeAndDispose()
   {
      super.closeAndDispose();
      
      jmeRenderer = null;
      
      if (panelViewports != null)
      {
         panelViewports.clear();
         panelViewports = null;
      }      
   }
}
