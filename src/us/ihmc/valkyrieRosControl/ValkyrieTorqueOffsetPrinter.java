package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.TorqueOffsetPrinter;

public class ValkyrieTorqueOffsetPrinter implements TorqueOffsetPrinter
{
   @Override
   public void printTorqueOffsets(DiagnosticsWhenHangingController diagnosticsWhenHangingController)
   {
      java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00;-0.00");

      System.out.println();

      ArrayList<OneDoFJoint> oneDoFJoints = diagnosticsWhenHangingController.getOneDoFJoints();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint);

         if (diagnosticsWhenHangingHelper != null)
         {
            double torqueOffset = diagnosticsWhenHangingHelper.getTorqueOffset();
            double torqueOffsetSign = diagnosticsWhenHangingController.getTorqueOffsetSign(oneDoFJoint);

            double signedTorqueOffset = torqueOffset * torqueOffsetSign;

            String offsetString = doubleFormat.format(signedTorqueOffset);
            System.out.println(oneDoFJoint.getName() + " torque offset = " + offsetString);
         }
      }

      System.out.println();
   }
}