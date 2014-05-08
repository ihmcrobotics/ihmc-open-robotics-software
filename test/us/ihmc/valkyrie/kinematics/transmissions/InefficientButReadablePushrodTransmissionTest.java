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
      assertJacobianEquals(jacobian, -0.036669850390129814, -0.036669850390129814, -0.034103255644705614, 0.034103255644705614);

      pitch.set(0.2);
      roll.set(0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03551500125933365, -0.036205837706435796, -0.03180995759280454, 0.036403074011934336);

      pitch.set(-0.2);
      roll.set(0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03694455631171873, -0.03472681771323452, -0.029852013228228228, 0.03438175613706602);

      pitch.set(0.2);
      roll.set(-0.1);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.036205837706435796, -0.03551500125933365, -0.036403074011934336, 0.03180995759280454);

      pitch.set(0.35);
      roll.set(0.0);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03407251067601638, -0.03407251067601638, -0.03341287882267228, 0.03341287882267228);

      pitch.set(-0.35);
      roll.set(0.0);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.034395294858413916, -0.034395294858413916, -0.030329503551170835, 0.030329503551170835);

      pitch.set(0.0);
      roll.set(0.25);
      computeAndPrint(inefficientButReadablePushrodTransmission, pitch, roll, jacobian, scs);
      assertJacobianEquals(jacobian, -0.03768210089049092, -0.03538451165784025, -0.026781907555281453, 0.038140623493685646);

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
