package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;

import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchHost;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchHostList;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.HostsChangedListener;


public abstract class DispatchHostPanel extends JPanel implements KeyListener, MouseListener, HostsChangedListener, Runnable
{
   private static final long serialVersionUID = 7012002654849386804L;
   protected DispatchHostList dispatchHostList;
   protected ArrayList<DispatchHost> dispatchHosts = new ArrayList<DispatchHost>();

   protected final static int FONT_SIZE = 12;

   public DispatchHostPanel(DispatchHostList dispatchHostList)
   {
      this.dispatchHostList = dispatchHostList;

      // hostsChanged();

      // this.setBorder(new SoftBevelBorder(BevelBorder.LOWERED));

      this.addMouseListener(this);
      this.addKeyListener(this);

      Thread anim = new Thread(this);
      anim.start();
   }

   public void run()
   {
      while (true)
      {
         this.repaint();

         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }
      }
   }



   public void mousePressed(MouseEvent evt)
   {
   }

   public void mouseReleased(MouseEvent evt)
   {
   }

   public void mouseEntered(MouseEvent evt)
   {
   }

   public void mouseExited(MouseEvent evt)
   {
   }

   protected int selected = -1;
   protected DispatchHost selectedHost;

   public void mouseClicked(MouseEvent evt)
   {
      // System.out.println("Mouse Clicked");

      int y = evt.getY();
      if (y < 0)
         return;
      selected = (y - 12) / FONT_SIZE;

      if (selected > dispatchHosts.size())
      {
         selected = -1;
         selectedHost = null;
      }

      else if ((selected >= 0) && (selected < dispatchHosts.size()))
      {
         selectedHost = (DispatchHost) dispatchHosts.get(selected);
      }

      else
      {
         selected = -1;
         selectedHost = null;
      }

      // System.out.print("Selected Host: ");
      // if (selectedHost != null) System.out.println(selectedHost.getHostName());

      this.requestFocus();
      this.repaint();

      // See if double click:

      /*
       * if ((evt.getClickCount() == 2) && (selectedVariable != null))
       * {
       * if (doubleClickListener != null) doubleClickListener.doubleClicked(selectedVariable);
       * }
       */
   }


   public DispatchHost getSelectedHost()
   {
      return this.selectedHost;
   }

   public void keyTyped(KeyEvent evt)
   {
   }

   public void keyReleased(KeyEvent evt)
   {
   }

   public void keyPressed(KeyEvent evt)
   {
      int code = evt.getKeyCode();

      if (code == KeyEvent.VK_UP)
      {
         if (selected > 0)
            selected--;
      }

      else if (code == KeyEvent.VK_DOWN)
      {
         if (selected < dispatchHosts.size() - 1)
            selected++;
      }

      if ((selected < 0) || (selected > dispatchHosts.size()))
         selectedHost = null;
      else
         selectedHost = (DispatchHost) dispatchHosts.get(selected);

      this.repaint();
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
   }

}
