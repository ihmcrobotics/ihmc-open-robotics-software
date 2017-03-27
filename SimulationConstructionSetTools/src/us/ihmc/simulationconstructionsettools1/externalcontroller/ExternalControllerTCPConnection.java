package us.ihmc.simulationconstructionsettools1.externalcontroller;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

class ExternalControllerTCPConnection
{
   DataInputStream in;
   DataOutputStream os;
   ServerSocket initialConnection;

   Socket clientSocket;
   int id;

   int portNumber = 8085;
   boolean bindRandomPort = false;
   boolean DEBUG = false;
   byte[] tempInBuffer = new byte[1024];
   byte[] tempOutBuffer = new byte[1024];
   int numBytesInDouble = Double.SIZE / 8;

   public ExternalControllerTCPConnection()
   {
      waitForClient();

      if (DEBUG)
         System.out.println("Connection " + id + " established with: " + clientSocket);

      try
      {
         in = new DataInputStream(clientSocket.getInputStream());

         os = new DataOutputStream(clientSocket.getOutputStream());
      }
      catch (IOException e)
      {
         if (DEBUG)
            System.out.println(e);
      }

      if (DEBUG)
         System.out.println("numBytesInDouble " + Double.toString(numBytesInDouble));
   }

   private void waitForClient()
   {
      // find an open port number
      boolean locked = false;
      while (!locked)
      {
         try
         {
            if (bindRandomPort)
            {
               portNumber = (int) Math.round((Math.random() * 8000) + 1000);
            }

            initialConnection = new ServerSocket(portNumber);
            System.out.println("Opening TCP connection on port " + portNumber);
            locked = true;
         }
         catch (IOException e)
         {
            System.out.println("Could not listen on port " + portNumber);
            System.out.println("opening new port");
            bindRandomPort = true;
         }
      }

      // wait for client to connect to make a new tcp connection
      System.out.println("waiting for external controller to connect on port " + portNumber);

      try
      {
         clientSocket = initialConnection.accept();
      }
      catch (IOException e)
      {
         if (DEBUG)
            System.out.println("Accept failed: " + portNumber);
         System.exit(-1);
      }

   }

   public void sendStringToExternalController(String sendMessage)
   {
      if (DEBUG)
         System.out.println("sending a string to the external controller");

      try
      {
         if (DEBUG)
            System.out.println(sendMessage);
         os.writeBytes(sendMessage);
         os.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public String getStringFromExternalController()
   {
      if (DEBUG)
         System.out.println("getting a string from the external controller");

      try
      {
         String recievedString = in.readLine();
         if (DEBUG)
            System.out.println(recievedString);

         return recievedString;
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return null;
   }

   public void sendDoubleArrayToExternalController(double[] doubleArray)
   {
      long tempLongL = 0;
      int iL;

      if (DEBUG)
         System.out.println("\nsending a double[] to the external controller");


      if (tempOutBuffer.length < doubleArray.length * numBytesInDouble)
      {
         tempOutBuffer = new byte[doubleArray.length * numBytesInDouble];
      }

      for (iL = 0; iL < doubleArray.length; iL++)
      {
         tempLongL = Double.doubleToRawLongBits(doubleArray[iL]);

         for (int jL = 0; jL < 8; jL++)
         {
            tempOutBuffer[iL * numBytesInDouble + jL] = (byte) ((tempLongL >> (7 - jL) * 8) & 0xFF);
         }
      }

      try
      {
         if (DEBUG)
         {
            for (iL = 0; iL < doubleArray.length; iL++)
            {
               System.out.println(doubleArray[iL] + ",");
            }
         }

         os.write(tempOutBuffer, 0, doubleArray.length * numBytesInDouble);
      }
      catch (Exception xcp)
      {
         xcp.printStackTrace();
      }
   }

   public double[] getDoubleArrayFromExternalController(int size)
   {
      if (DEBUG)
         System.out.println("\ngetting a double[] from the external controller of size " + size);

      double[] darr = new double[size];

/*
      //    DON'T UNDERSTAND WHY THIS FAILS, BUT CONVERSION TO DOUBLE IS THE PROBLEM.  IT SHOULD WORK BASED ON DOCS, BUT DOESN'T
            long tempLongL;

            if (tempInBuffer.length < size * numBytesInDouble)
            {
               tempInBuffer = new byte[size * numBytesInDouble];
            }

            try
            {
               while (in.available() < size * numBytesInDouble)
               {
                  Thread.sleep(1);
               }

               if (in.read(tempInBuffer, 0, size * numBytesInDouble) != size * numBytesInDouble)
               {
                  System.out.println("Could not read whole packet");

                  return darr;
               }

               for (int iL = 0; iL < size; iL++)
               {
                  tempLongL = 0;

                  for (int jL = 0; jL < 8; jL++)
                  {
                     tempLongL |= tempInBuffer[iL * numBytesInDouble + jL] << ((7 - jL) * 8);
                  }

                  darr[iL] = Double.longBitsToDouble(tempLongL);
               }
            }
            catch (Exception xcp)
            {
               xcp.printStackTrace();
            }
*/


      try
      {
         while (in.available() <= 0)
         {
            try
            {
               Thread.sleep(1);
            }
            catch (InterruptedException e)
            {
            }
         }

         for (int i = 0; i < size; i++)
         {
            try
            {
               darr[i] = in.readDouble();
               if (DEBUG)
                  System.out.print(darr[i] + ",");

            }
            catch (Exception xcp)
            {
               break;
            }
         }

      }
      catch (IOException e)
      {
         e.printStackTrace();
      }


      return darr;
   }

   public void close()
   {
      try
      {
         clientSocket.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
