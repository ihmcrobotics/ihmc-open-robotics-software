package us.ihmc.simulationconstructionset.whiteBoard;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.thread.ThreadTools;


public class TCPYoWhiteBoardEvaluation
{
   private static final boolean VERBOSE = false;
   private static final boolean USE_UDP = true;
   private static final boolean THROW_OUT_STALE_PACKETS = true;

   public TCPYoWhiteBoardEvaluation(boolean leftSide, int port, DoSomeWorker doSomeWorker,
                                    int numberVariablesToReadLeftWriteRight, int numberVariablesToWriteLeftReadRight)
           throws UnknownHostException, IOException
   {
      this(leftSide, null, port, doSomeWorker, numberVariablesToReadLeftWriteRight, numberVariablesToWriteLeftReadRight);
   }

   public TCPYoWhiteBoardEvaluation(boolean leftSide, String ipAddress, int port, DoSomeWorker doSomeWorker,
                                    int numberVariablesToReadLeftWriteRight, int numberVariablesToWriteLeftReadRight)
           throws UnknownHostException, IOException
   {
      YoWhiteBoard whiteBoard;

      if (leftSide)
      {
         if (USE_UDP)
         {
            if ((ipAddress == null) || (ipAddress.equals("null")))
               throw new RuntimeException("Need a real ipAddress when making a UDP white board evaluation. ipAddress = " + ipAddress);
            whiteBoard = new UDPYoWhiteBoard("udpLeft", true, ipAddress, port, THROW_OUT_STALE_PACKETS);
         }
         else
         {
            if ((ipAddress != null) && (!ipAddress.equals("null")))
               throw new RuntimeException("Left side should have null ipAddress when making a white board evaluation. ipAddress = " + ipAddress);
            whiteBoard = new TCPYoWhiteBoard("tcpLeft", port);
         }
      }
      else
      {
         if (USE_UDP)
         {
            if ((ipAddress == null) || (ipAddress.equals("null")))
               throw new RuntimeException("Need a real ipAddress when making a UDP white board evaluation. ipAddress = " + ipAddress);
            whiteBoard = new UDPYoWhiteBoard("udpRight", false, ipAddress, port, THROW_OUT_STALE_PACKETS);

         }
         else
         {
            if (ipAddress == null)
               throw new RuntimeException("Right side should not have null ipAddress when making a white board evaluation.");
            whiteBoard = new TCPYoWhiteBoard("tcpRight", ipAddress, port);
         }
      }

      ThreadTools.startAThread((Runnable) whiteBoard, "TCPYoWhiteBoardEvaluationThread");

      ArrayList<YoVariable<?>> variablesToReadLeftWriteRight = generateSomeDoubleYoVariables("readLeftWriteRight", numberVariablesToReadLeftWriteRight);
      ArrayList<YoVariable<?>> variablesToWriteLeftReadRight = generateSomeDoubleYoVariables("writeLeftReadRight", numberVariablesToWriteLeftReadRight);

      if (leftSide)
      {
         whiteBoard.setVariablesToRead(variablesToReadLeftWriteRight);
         whiteBoard.setVariablesToWrite(variablesToWriteLeftReadRight);
      }

      else
      {
         whiteBoard.setVariablesToRead(variablesToWriteLeftReadRight);
         whiteBoard.setVariablesToWrite(variablesToReadLeftWriteRight);
      }

      if (VERBOSE)
         System.out.println("TCPYoWhiteBoardEvaluation Connecting");

      whiteBoard.connect();

      if (VERBOSE)
         System.out.println("TCPYoWhiteBoardEvaluation Waiting for whiteBoard to be connected");

      while (!whiteBoard.isConnected())
      {
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException e)
         {
         }
      }

      if (VERBOSE)
         System.out.println("TCPYoWhiteBoardEvaluation WhiteBoard is now connected");

      int numberOfLoops = 0;
      double maxLoopTime = Double.NEGATIVE_INFINITY;

      long startStartTime = System.nanoTime();


      String sideText;
      if (leftSide)
         sideText = "Left";
      else
         sideText = "Right";

      while (true)
      {
         long loopStartTime = System.nanoTime();

         if (leftSide)
         {
            randomizeVariables(variablesToWriteLeftReadRight);
            whiteBoard.writeData();

            // Do some work here...
            if (doSomeWorker != null)
            {
               doSomeWorker.doSomeWork();
            }

            if (!USE_UDP)
            {
               while (!whiteBoard.isNewDataAvailable())
               {
                  Thread.yield();
               }
            }

            int numberOfNewData = whiteBoard.getNumberOfNewDataSinceLastRead();
            whiteBoard.readData();
         }


         else
         {
//          if (!USE_UDP)
            {
               while (!whiteBoard.isNewDataAvailable())
               {
                  Thread.yield();
               }
            }

            int numberOfNewData = whiteBoard.getNumberOfNewDataSinceLastRead();
            whiteBoard.readData();

            // Do some work here...
            if (doSomeWorker != null)
            {
               doSomeWorker.doSomeWork();
            }

            randomizeVariables(variablesToReadLeftWriteRight);

            whiteBoard.writeData();
         }

         numberOfLoops++;
         long currentTime = System.nanoTime();
         double loopDiffTime = (currentTime - loopStartTime) * 1.0e-9;

         if (loopDiffTime > maxLoopTime)
         {
            maxLoopTime = loopDiffTime;
         }

         double totalDiffTime = (currentTime - startStartTime) * 1.0e-9;
         double averageLoopTime = totalDiffTime / ((double) numberOfLoops);

         if (numberOfLoops > 5000)
         {
            System.out.println(sideText + " averageLoopTime in last bunch = " + averageLoopTime + ", maxLoopTime in last bunch = " + maxLoopTime);
            System.out.flush();
            maxLoopTime = Double.NEGATIVE_INFINITY;

            startStartTime = System.nanoTime();
            numberOfLoops = 0;
         }
      }
   }


   private void randomizeVariables(ArrayList<YoVariable<?>> variablesToWrite)
   {
      for (YoVariable<?> variable : variablesToWrite)
      {
         variable.setValueFromDouble(Math.random());
      }
   }

   public ArrayList<YoVariable<?>> generateSomeDoubleYoVariables(String name, int numberOfVariables)
   {
      YoVariableRegistry registry = new YoVariableRegistry("root");

      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      for (int i = 0; i < numberOfVariables; i++)
      {
         DoubleYoVariable variable = new DoubleYoVariable(name + i, registry);
         variable.set(Math.random());
         ret.add(variable);
      }

      return ret;
   }

   public static void main(String[] args) throws UnknownHostException, IOException
   {
      int port = 7799;
      int numberVariablesOne = 200;
      int numberVariablesTwo = 100;
      
      if ((args == null) || (args.length == 0))
      {
         System.out.println("Starting a Server. If you want a client, put the ipAddress in the program arguments.");
         
         DoSomeWorker doSomeWorker = new PauseDoSomeWorker();
         
         
         new TCPYoWhiteBoardEvaluation(true, port, doSomeWorker, numberVariablesOne, numberVariablesTwo);
      }

      else
      {
         String ipAddress = args[0];
         System.out.println("Starting a client, attaching to " + ipAddress + ". If you want a server, don't put the ipAddress in the program arguments.");

         DoSomeWorker doSomeWorker = new PauseDoSomeWorker();
         new TCPYoWhiteBoardEvaluation(false, ipAddress, port, doSomeWorker, numberVariablesOne, numberVariablesTwo);
      }
   }
   
   private static class PauseDoSomeWorker implements DoSomeWorker
   {

      @Override
      public void doSomeWork()
      {
         try
         {
            Thread.sleep(2);
         } 
         catch (InterruptedException e)
         {
         }
      }
      
   }
}
