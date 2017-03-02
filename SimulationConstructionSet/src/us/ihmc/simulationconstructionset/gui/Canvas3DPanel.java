package us.ihmc.simulationconstructionset.gui;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.border.AbstractBorder;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;


public class Canvas3DPanel extends JPanel implements MouseListener
{
   private static final long serialVersionUID = -4186691483384418845L;
   private ViewportAdapterAndCameraControllerHolder view;

   private ViewportPanel viewportPanel;
   private boolean active = false;

   private AbstractBorder border;
   private Canvas canvas;
   private SpaceKeyListener spaceKeyListener;
   
   public Canvas3DPanel(RunCommandsExecutor runCommandsExecutor, ViewportAdapterAndCameraControllerHolder viewportAdapterAndCameraControllerHolder, ViewportPanel viewportPanel)
   {
      super(new GridLayout(1, 1));
      this.view = viewportAdapterAndCameraControllerHolder;
      this.viewportPanel = viewportPanel;

      canvas = viewportAdapterAndCameraControllerHolder.getViewportAdapter().getCanvas();
      this.add(canvas);

      this.setRequestFocusEnabled(true);

      canvas.addMouseListener(this);

      Border raisedbevel = BorderFactory.createRaisedBevelBorder();
      Border loweredbevel = BorderFactory.createLoweredBevelBorder();
      border = BorderFactory.createCompoundBorder(raisedbevel, loweredbevel);

      this.setBorder(border);
      spaceKeyListener = new SpaceKeyListener(runCommandsExecutor);
      canvas.addKeyListener(spaceKeyListener);
   }

   public ViewportAdapterAndCameraControllerHolder getStandard3DView()
   {
      return view;
   }

   @Override
   public void mouseClicked(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }

   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }

   @Override
   public void mousePressed(MouseEvent evt)
   {
      // System.out.println("Mouse Pressed in Canvas3DPanel!" + view); //.getCamera());
      // standardSimulationGraphics.setActiveView(view, this);
      viewportPanel.setActiveView(view, this);
   }


   public void setActive(boolean active)
   {
      if (this.active == active)
         return;
      this.active = active;

      Border raisedbevel = BorderFactory.createRaisedBevelBorder();
      Border loweredbevel = BorderFactory.createLoweredBevelBorder();

      // border = BorderFactory.createCompoundBorder(loweredbevel, raisedbevel);
      // border = BorderFactory.createCompoundBorder(raisedbevel, redline);
      // border = new TitledBorder("hello");

      if (active)
      {
         Border redline = BorderFactory.createLineBorder(Color.red);
         border = BorderFactory.createCompoundBorder(redline, loweredbevel);
      }

      else
      {
         border = BorderFactory.createCompoundBorder(raisedbevel, loweredbevel);
      }

      this.setBorder(border);
   }


   private class SpaceKeyListener implements KeyListener
   {
      private RunCommandsExecutor runCommandsExecutor;

      public SpaceKeyListener(RunCommandsExecutor runCommandsExecutor)
      {
         this.runCommandsExecutor = runCommandsExecutor;
      }

      @Override
      public void keyTyped(KeyEvent keyEvent)
      {
         if (keyEvent.getKeyChar() == ' ')
         {
            if (runCommandsExecutor.isSimulating())
            {
               runCommandsExecutor.stop();
            }
            else
               runCommandsExecutor.simulate();
         }

      }

      @Override
      public void keyReleased(KeyEvent e)
      {
      }

      @Override
      public void keyPressed(KeyEvent e)
      {
      }

      public void closeAndDispose()
      {
         this.runCommandsExecutor = null;
      }
   }
   
   public void closeAndDispose()
   {
      this.removeAll();

      if (canvas != null)
      {
         canvas.removeKeyListener(spaceKeyListener);
         canvas = null;
      }

      if (spaceKeyListener != null)
      {
         spaceKeyListener.closeAndDispose();
      }

      view = null;
      viewportPanel = null;
      border =  null;
   }
}
