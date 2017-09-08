package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;

import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.KickBallBehaviorCoactiveElement;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleCoactiveElementUserInterface
{
   private final CoactiveElement coactiveElement;
   private SimulationConstructionSet scs;

   public SimpleCoactiveElementUserInterface(CoactiveElement coactiveElement)
   {
      this.coactiveElement = coactiveElement;
   }

   public void startOnAThread(String ipAddress, final boolean showSCS)
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

      CoactiveElementYoWhiteBoardSynchronizer synchronizer = new CoactiveElementYoWhiteBoardSynchronizer(NetworkPorts.COACTIVE_ELEMENTS_PORT.getPort(), ipAddress,
                                                                                                         HumanOrMachine.HUMAN, coactiveElement);

      final long millisecondsBetweenDataWrites = 300L;
      synchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);

      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
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

         @Override public void initializeMachineSide()
         {
         }

         @Override public void updateMachineSide()
         {
         }
      };

      //      SimpleCoactiveElementMachineSide machineSide = new SimpleCoactiveElementMachineSide(coactiveElement);
      //      machineSide.startOnAThread(PORT_FOR_COACTIVE_ELEMENTS);

      SimpleCoactiveElementUserInterface userInterface = new SimpleCoactiveElementUserInterface(UISide);
      userInterface.startOnAThread("localhost", true);
   }

}
