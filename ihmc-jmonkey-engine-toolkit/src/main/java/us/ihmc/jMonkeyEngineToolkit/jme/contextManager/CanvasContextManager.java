package us.ihmc.jMonkeyEngineToolkit.jme.contextManager;

import java.awt.Canvas;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import com.jme3.input.MouseInput;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.math.Vector2f;

import us.ihmc.jMonkeyEngineToolkit.jme.InputMapSetter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMECamera;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEContextManager;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEViewportAdapter;

public class CanvasContextManager extends JMEContextManager implements InputMapSetter, AnalogListener, MouseListener
{
   private JMERenderer jmeRenderer;
   
   
   public CanvasContextManager(JMERenderer jmeRenderer)
   {
      super(jmeRenderer);
      
      this.jmeRenderer = jmeRenderer;
      registerInputMapSetter(this);
   }

   @Override
   public void focusOnCurrentWindow()
   {
      jmeRenderer.getCanvas().requestFocus();
   }

   @Override
   protected void addJMEViewportAdapterToContext(JMEViewportAdapter jmeViewportAdapter)
   {
   }

   public void setDefaultInputMappings()
   {
      jmeRenderer.getInputManager().addMapping("CanvasMouseRight", new MouseAxisTrigger(MouseInput.AXIS_X, false));
      jmeRenderer.getInputManager().addMapping("CanvasMouseLeft", new MouseAxisTrigger(MouseInput.AXIS_X, true));
      jmeRenderer.getInputManager().addMapping("CanvasMouseUp", new MouseAxisTrigger(MouseInput.AXIS_Y, true));
      jmeRenderer.getInputManager().addMapping("CanvasMouseDown", new MouseAxisTrigger(MouseInput.AXIS_Y, false));
      
      
      jmeRenderer.getInputManager().addListener(this, new String[]{"CanvasMouseLeft", "CanvasMouseRight", "CanvasMouseUp", "CanvasMouseDown"});
   }
   
   private void setViewport()
   {
      if(isSwitchingEnabled())
      {
         Vector2f cursorPosition = jmeRenderer.getInputManager().getCursorPosition();
         
         Canvas canvas = jmeRenderer.getCanvas();
         float height = canvas.getHeight();
         float width = canvas.getWidth();
         
         for(int i = 0; i < viewports.size(); i++)
         {
            JMEViewportAdapter viewport = viewports.get(i);
            JMECamera camera = viewport.getCamera();
            float xStart = camera.getViewPortLeft() * width;
            float xEnd = camera.getViewPortRight() * width;
            float yStart = camera.getViewPortBottom() * height;
            float yEnd = camera.getViewPortTop() * height;
            
            
            if(xStart < cursorPosition.x &&
                xEnd > cursorPosition.x &&
                yStart < cursorPosition.y && 
                yEnd > cursorPosition.y)
            {
               if(viewport != getCurrentViewport())
               {
                  resetViewport(getCurrentViewport());
                  setCurrentViewport(viewport);
               }
               return;
            }
            
         }
      }
   }
   
   public void onAnalog(String name, float value, float tpf) 
   {
      setViewport();
   }

   public void reset()
   {
   }

   @Override
   public void initialize()
   {
      setDefaultInputMappings();
      jmeRenderer.getCanvas().addMouseListener(this);
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
      setViewport();
   }

   public void mouseExited(MouseEvent e)
   {
      resetViewport(getCurrentViewport());
   }
   
   public void closeAndDispose()
   {
      super.closeAndDispose();
      
      jmeRenderer = null;
   }


}
