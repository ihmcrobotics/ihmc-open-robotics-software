package us.ihmc.systemIdentification.com;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JOptionPane;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.buffer.YoBuffer;

public class LinkComIDActionListener implements ActionListener
{

   private String[] linkNames;
   private YoBuffer dataBuffer;
   private final Robot robot;

   public LinkComIDActionListener(YoBuffer dataBuffer, Robot robot)
   {
      this.dataBuffer = dataBuffer;
      this.robot = robot;

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);
      this.linkNames = new String[joints.size()];
      for (int i = 0; i < linkNames.length; i++)
      {
         linkNames[i] = joints.get(i).getLink().getName();
      }
   }

   public void actionPerformed(ActionEvent arg0)
   {
      Thread foo = new Thread(new Runnable()
      {
         final Robot localRobot = robot;

         @Override
         public void run()
         {
            //select joint
            String targetLink = (String) JOptionPane.showInputDialog(null, "choose body", "choose body", JOptionPane.QUESTION_MESSAGE, null, linkNames,
                  linkNames[0]);
            if (targetLink == null)
            {
               System.err.println("canceled link selection");
               return;
            }

            //check data
            if (dataBuffer.getInPoint() == 0 && dataBuffer.getOutPoint() == 0)
            {
               System.err.println("Please load data before calibration");
               return;
            }

            if (dataBuffer.findVariable("sensedCoPX") == null)
            {
               System.err.println("data must contain sensedCoPX,Y,Z for calibration");
               return;
            }
            System.out.println("IN/out" + dataBuffer.getInPoint() + " out " + dataBuffer.getOutPoint() + " " + dataBuffer.getBufferInOutLength());

            ComCopResidual residual = new ComCopResidual(localRobot, targetLink, dataBuffer, dataBuffer.getKeyPointsHandler().getKeyPoints().size() > 0 ? -1 : 1000);
            LinkComID optimizer = new LinkComID(residual);
            residual.showSample(20, "PreOpt CoM: " + residual.getCurrentLinkCom());

            optimizer.optimize();
            residual.showSample(20, "PostOpt CoM: " + residual.getCurrentLinkCom());
         }

      }, "IHMC-LinkComIDActionListener");
      foo.start();

   }

}
