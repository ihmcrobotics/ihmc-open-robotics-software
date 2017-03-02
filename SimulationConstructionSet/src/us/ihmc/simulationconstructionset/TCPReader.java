package us.ihmc.simulationconstructionset;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;


class TCPReader extends Thread
{
   private Socket _socket;
   long starttime = 0;
   int serverPort = 8103;
   Socket s = null;

   public TCPReader(Socket s)
   {
      _socket = s;
      System.out.println("Connected to socket on port " + serverPort);
      starttime = System.currentTimeMillis();
   }

   @Override
   public void run()
   {
      try
      {
         DataInputStream in = new DataInputStream(_socket.getInputStream());

         boolean stop = false;
         int datasize = 0;

         int processed = 0;
         while (!stop)
         {
            double[] darr = new double[5];
            stop = true;

            while (in.available() <= 0)
            {
               try
               {
                  sleep(1);
               }
               catch (InterruptedException e)
               {
               }
            }

            for (int i = 0; i < 5; i++)
            {
               try
               {
                  darr[i] = in.readDouble();
                  if (darr[i] != -1.0)
                     stop = false;
               }
               catch (EOFException xcp)
               {
                  if (!stop)
                  {
                     xcp.printStackTrace();
                     stop = true;

                     break;
                  }
               }

               // System.out.println(i + " " + darr[i]);
            }

//          for (int i = 58; i < 64; i++ )
//          {
//              darr[i] = 0.0;
//          }
            processed++;
            System.out.println("Processed " + processed + " rows of data");

            // do something with the data
//          addToReceivedData(darr);
         }

         long diff = System.currentTimeMillis() - starttime;
         System.out.println("Reading took " + (diff / 1000.0) + " secs.");

         in.close();
         _socket.close();
         _socket = null;
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      Socket s;
      try
      {
         ServerSocket myServer = new ServerSocket(8103);

         s = new Socket("10.100.0.177", 8103);
         System.out.println(s.getInetAddress().getHostName());
         TCPReader reader1 = new TCPReader(s);
         reader1.run();
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }


   }

}
