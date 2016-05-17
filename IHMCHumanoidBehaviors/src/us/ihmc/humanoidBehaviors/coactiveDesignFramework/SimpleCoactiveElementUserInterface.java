package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.JButton;
import javax.swing.JFrame;

import us.ihmc.humanoidBehaviors.behaviors.KickBallBehaviorCoactiveElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;

public class SimpleCoactiveElementUserInterface
{
   private static final int PORT_FOR_COACTIVE_ELEMENTS = 56122;
   private final CoactiveElement coactiveElement;
   private SimulationConstructionSet scs;

   public SimpleCoactiveElementUserInterface(CoactiveElement coactiveElement)
   {
      this.coactiveElement = coactiveElement;
   }

   public void startOnAThread(String ipAddress, int port, final boolean showSCS)
   {
      if (showSCS)
      {
         scs = new SimulationConstructionSet(new Robot("NullRobotUserInterfaceSide"));
         YoVariableRegistry rootRegistry = scs.getRootRegistry();
         rootRegistry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());
         rootRegistry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
         scs.startOnAThread();
      }
      coactiveElement.initializeUserInterfaceSide();
      final TCPYoWhiteBoard userInterfaceSideWhiteBoard = new TCPYoWhiteBoard("UserInterfaceSideWhiteBoard", ipAddress, port);

      CoactiveElementYoWhiteBoardSynchronizer userInterfaceSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(userInterfaceSideWhiteBoard,
            HumanOrMachine.HUMAN, coactiveElement);

      Thread userInterfaceSideThread = new Thread(userInterfaceSideWhiteBoard);
      userInterfaceSideThread.start();

      final long millisecondsBetweenDataWrites = 300L;
      userInterfaceSideSynchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);

      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            boolean connected = false;

            while (!connected)
            {
               try
               {
                  System.out.println("Trying to connect");
                  userInterfaceSideWhiteBoard.connect();
                  connected = true;
                  System.out.println("Connected!!!");
               }
               catch (IOException e1)
               {
                  System.out.println(e1);
                  sleep(1000L);
               }
            }

            while (true)
            {
               coactiveElement.updateUserInterfaceSide();

               if (showSCS)
                  scs.tickAndUpdate();

               sleep(millisecondsBetweenDataWrites);
            }
         }
      };

      Thread thread = new Thread(runnable);
      thread.start();
   }

   private void sleep(long sleepTimeMillis)
   {
      try
      {
         Thread.sleep(sleepTimeMillis);
      }
      catch (Exception e)
      {
      }
   }

   public static void main(String[] args)
   {
      KickBallBehaviorCoactiveElement UISide = new KickBallBehaviorCoactiveElement()
      {
         private JFrame jFrame;
         public void initializeUserInterfaceSide()
         {
            if (jFrame == null)
            {
               jFrame = new JFrame();
               jFrame.setSize(600, 600);

               JButton jButton = new JButton("Abort");
               jFrame.getContentPane().add(jButton);

               jButton.addActionListener(new ActionListener()
               {
                  @Override
                  public void actionPerformed(ActionEvent arg0)
                  {
                     abortClicked.set(true);
                  }
               });

               jFrame.setVisible(true);
            }
            userInterfaceSideCount.set(10);
         }

         @Override
         public void updateUserInterfaceSide()
         {
            userInterfaceSideCount.increment();

            if ((abortClicked.getBooleanValue()) && (abortAcknowledged.getBooleanValue()))
            {
               abortClicked.set(false);
            }
         }
      };

      //      SimpleCoactiveElementMachineSide machineSide = new SimpleCoactiveElementMachineSide(coactiveElement);
      //      machineSide.startOnAThread(PORT_FOR_COACTIVE_ELEMENTS);

      SimpleCoactiveElementUserInterface userInterface = new SimpleCoactiveElementUserInterface(UISide);
      userInterface.startOnAThread("localhost", PORT_FOR_COACTIVE_ELEMENTS, true);
   }

}
