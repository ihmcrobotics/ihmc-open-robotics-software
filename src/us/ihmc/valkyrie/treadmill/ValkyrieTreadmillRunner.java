package us.ihmc.valkyrie.treadmill;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import javax.swing.JFrame;

public class ValkyrieTreadmillRunner
{

   private static final String TREADMILL_IP = "192.168.2.212";
   private static final int TREADMILL_PORT = 5151; 
   
   public static MAVLink mav = new MAVLink();
   public static java.io.InputStream inputStream = null;

   public static volatile double duty = 0;
   private static Socket socket = null;
   
   public ValkyrieTreadmillRunner() throws UnknownHostException, IOException
   {
      //TODO: Figure out how to do this
   }

   public static void main(String[] args)
   {
      new Frame();
      try
      {
         socket = new Socket(TREADMILL_IP, TREADMILL_PORT);
         java.io.OutputStream outStream = socket.getOutputStream();
         inputStream = socket.getInputStream();
         new Listener().start();

         while (true)
         {
            outStream.write(new SetPWM(1, 1, duty).pack());
            outStream.write(new GetVelocity(1, 1).pack());

            Thread.sleep(1000);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
      if (socket != null)
      {
         try
         {
            socket.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public static class Listener extends Thread
   {
      public void run()
      {
         while (true)
         {
            if (inputStream != null)
            {
               try
               {
                  while (inputStream.available() > 0)
                  {
                     int r = inputStream.read();
                     if (mav.parseChar((byte) r))
                     {
                        if (mav.lastMsg.getMessageID() == 2)
                        {
                           System.out.println("Velocity: " + (new SetVelocity(mav.lastMsg)).getVelocity());
                        }
                     }
                  }
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
         }
      }
   }

   public static class Frame extends JFrame implements KeyListener
   {

      private static final long serialVersionUID = -862241579825174141L;

      public Frame()
      {
         this.addKeyListener(this);
         this.setBounds(0, 0, 50, 50);
         this.setResizable(false);
         this.setVisible(true);
      }

      @Override
      public void keyPressed(KeyEvent e)
      {
         switch (e.getKeyCode())
         {
         case 107:
            duty += 1;
            if (duty > 100)
               duty = 100;
            break;
         case 109:
            duty -= 1;
            if (duty < 0)
               duty = 0;
            break;
         default:
            break;
         }
         System.out.println("Duty: " + duty);
      }

      @Override
      public void keyReleased(KeyEvent e)
      {

      }

      @Override
      public void keyTyped(KeyEvent e)
      {

      }
   }
}
