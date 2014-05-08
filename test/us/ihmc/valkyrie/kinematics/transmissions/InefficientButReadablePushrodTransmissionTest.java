package us.ihmc.valkyrie.kinematics.transmissions;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class InefficientButReadablePushrodTransmissionTest
{
   private final boolean visualizeAndKeepUp = false;

   @Test
   public void testInefficientButReadablePushrodTransmission()
   {
      Robot robot = new Robot("testPushrodTransmission");

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
      if (visualizeAndKeepUp)
      {
         dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      }
      else
      {
         dynamicGraphicObjectsListRegistry = null;
      }

      InefficientButReadablePushrodTransmission inefficientButReadablePushrodTransmission = new InefficientButReadablePushrodTransmission(registry,
                                                                                               dynamicGraphicObjectsListRegistry);

      SimulationConstructionSet scs;
      if (visualizeAndKeepUp)
      {
         scs = new SimulationConstructionSet(robot);
         dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
         scs.setCameraPosition(0.62, -0.4, 1.26);
         scs.setCameraFix(0.0, 0.0, 1.02);

         scs.startOnAThread();
      }
      else
      {
         scs = null;
      }

      double[][] jacobian = new double[2][2];

      if (visualizeAndKeepUp)
      {
         double increment = 0.1;
         for (pitch.set(-Math.PI / 3.0); pitch.getDoubleValue() < Math.PI / 3.0; pitch.add(increment))
         {
            for (roll.set(-Math.PI / 3.0); roll.getDoubleValue() < Math.PI / 3.0; roll.add(increment))
            {
               computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
            }
         }
      }

      pitch.set(0.0);
      roll.set(0.0);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.0366712094326246, -0.0366712094326246, 0.034118686505983736, 0.034118686505983736);

      pitch.set(0.2);
      roll.set(0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.036202355875729446, -0.035509962933305175, 0.03640538461195576, 0.03640538461195576);

      pitch.set(-0.2);
      roll.set(0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.034740329545336665, -0.03695254741929382, 0.03440182578269918, 0.03440182578269918);

      pitch.set(0.2);
      roll.set(-0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.035509962933305175, -0.036202355875729446, 0.03181607828356141, 0.03181607828356141);

      pitch.set(0.35);
      roll.set(0.0);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03406729338743576, -0.03406729338743576, 0.03341354644555879, 0.03341354644555879);

      pitch.set(-0.35);
      roll.set(0.0);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03440991379530292, -0.03440991379530292, 0.030355910924449715, 0.030355910924449715);

      pitch.set(0.0);
      roll.set(0.25);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03539813540952868, -0.037679153131957736, 0.038150540900731125, 0.038150540900731125);

      if (visualizeAndKeepUp)
      {
         scs.gotoInPointNow();
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();

         ThreadTools.sleepForever();
      }

   }


   private void assertJacobianEquals(double[][] jacobian, double j00, double j01, double j10, double j11)
   {
      assertEquals(jacobian[0][0], j00, 1e-7);
      assertEquals(jacobian[0][1], j01, 1e-7);
      assertEquals(jacobian[1][0], j10, 1e-7);
      assertEquals(jacobian[1][1], j11, 1e-7);

   }

   private void computeAndPrint(InefficientButReadablePushrodTransmission inefficientButReadablePushrodTransmission, DoubleYoVariable pitch,
                                DoubleYoVariable roll, double[][] jacobian, SimulationConstructionSet scs)
   {
      inefficientButReadablePushrodTransmission.computeJacobian(jacobian, pitch.getDoubleValue(), roll.getDoubleValue());

      if (visualizeAndKeepUp)
      {
         scs.tickAndUpdate();

         System.out.println(pitch + ", " + roll + " f5=1 -> tauPitch = " + jacobian[0][0] + ", tauRoll = " + jacobian[1][0]);
         System.out.println(pitch + ", " + roll + " f6=1 -> tauPitch = " + jacobian[0][1] + ", tauRoll = " + jacobian[1][1] + "\n");
      }
   }
}
