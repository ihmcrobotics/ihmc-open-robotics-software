package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;

import us.ihmc.simulationconstructionset.NewDataListener;


public class GUISideProtocolListenerThread implements Runnable
{
   private static final boolean DEBUG = true;

   private GUISideProtocolListener guiSideProtocolListener;
   private Thread anim;
   private boolean alive = true;
   private DataOutputStream dataOut;
   private DataInputStream dataIn;

   private GUISideAbstractCommandListener commandListener;


   public GUISideProtocolListenerThread(Socket socket, GUISideAbstractCommandListener commandListener)
   {
      this(socket, commandListener, null);
   }

   public GUISideProtocolListenerThread(Socket socket, GUISideAbstractCommandListener commandListener, ArrayList<NewDataListener> newDataListeners)
   {
      try
      {
         socket.setSoTimeout(10000);
      }    // Timeout of the socket in miliseconds.
      catch (SocketException se)
      {
         System.err.println(se.getStackTrace().toString());
      }

      this.commandListener = commandListener;

//    this.sendIndexMap = sendIndexMap;
      System.out.println("Creating RobotProtocolListener");

      try
      {
         System.out.println("Creating DataInputStream");
         dataIn = new DataInputStream(new BufferedInputStream(socket.getInputStream()));

         // dataIn = new RecordedDataInputOutputStream(new DataInputStream(new BufferedInputStream(socket.getInputStream())),
         // new DataOutputStream(new BufferedOutputStream(socket.getOutputStream())));

         System.out.println("Creating DataOutputStream");
         dataOut = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));

         guiSideProtocolListener = new GUISideProtocolListener(dataIn, commandListener, newDataListeners);

         System.out.println("Creating and Starting RobotProtocolListenerThread");
         anim = new Thread(this);

         // anim.setPriority(anim.getPriority()+1);
         anim.start();

         // System.out.println("Creating Recorded Stream Frame");
         // RecordedStreamFrame frame = new RecordedStreamFrame();

      }
      catch (IOException e)
      {
         e.printStackTrace();
         alive = false;
      }
   }

   public DataOutputStream getDataOutputStream()
   {
      return dataOut;
   }
   
   public GUISideProtocolListener getGUISideProtocolListener()
   {
      return guiSideProtocolListener;
   }

   public void disconnect()
   {
      if (alive)
      {
         this.alive = false;
         this.anim = null;

         commandListener.doDisconnect();
      }

      try
      {
         dataIn.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         dataOut.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      alive = false;
      anim = null;
   }


   public boolean isAlive()
   {
      return this.alive;
   }

   private boolean pause = false;

   public void pause()
   {
      pause = true;
   }

   @Override
   public void run()
   {
      while (anim == Thread.currentThread())
      {
         try
         {
            guiSideProtocolListener.processInput();

            if (pause)
            {
               try
               {
                  Thread.sleep(2000);
               }
               catch (InterruptedException e)
               {
               }

               pause = false;
            }

         }
         catch (IOException e)
         {
            System.out.println("Caught IOexception in RobotPrototcolListenerThread " + e);

            if (DEBUG)
            {
               e.printStackTrace();
               System.exit(-1);
            }

            System.out.println("Disconnecting");
            disconnect();
         }
      }
   }
}
