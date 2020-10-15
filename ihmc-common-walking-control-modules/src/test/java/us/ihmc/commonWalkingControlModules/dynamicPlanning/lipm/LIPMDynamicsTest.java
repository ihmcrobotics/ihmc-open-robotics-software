package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;

public class LIPMDynamicsTest
{
   @Test
   public void testNextStateComputation()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 0);

      DMatrixRMaj nextState = new DMatrixRMaj(6, 1);

      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, nextState);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      double nextX = x + deltaT * xDot + 0.5 * deltaT * deltaT * (x - pX) * fz / (mass * z);
      double nextY = y + deltaT * yDot + 0.5 * deltaT * deltaT * (y - pY) * fz / (mass * z);
      double nextZ = z + deltaT * zDot + 0.5 * deltaT * deltaT * (fz / mass - gravityZ);
      double nextXDot = xDot + deltaT * (x - pX) * fz / (mass * z);
      double nextYDot = yDot + deltaT * (y - pY) * fz / (mass * z);
      double nextZDot = zDot + deltaT * (fz / mass - gravityZ);

      DMatrixRMaj expectedNextState = new DMatrixRMaj(6, 1);
      expectedNextState.set(0, nextX);
      expectedNextState.set(1, nextY);
      expectedNextState.set(2, nextZ);
      expectedNextState.set(3, nextXDot);
      expectedNextState.set(4, nextYDot);
      expectedNextState.set(5, nextZDot);


      MatrixTestTools.assertMatrixEquals(expectedNextState, nextState, 1e-10);

      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         nextState = new DMatrixRMaj(6, 1);

         dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, nextState);

         assertEquals(6, dynamics.getStateVectorSize());
         assertEquals(3, dynamics.getControlVectorSize());

         nextX = x + deltaT * xDot + 0.5 * deltaT * deltaT * (x - pX) * fz / (mass * z);
         nextY = y + deltaT * yDot + 0.5 * deltaT * deltaT * (y - pY) * fz / (mass * z);
         nextZ = z + deltaT * zDot + 0.5 * deltaT * deltaT * (fz / mass - gravityZ);
         nextXDot = xDot + deltaT * (x - pX) * fz / (mass * z);
         nextYDot = yDot + deltaT * (y - pY) * fz / (mass * z);
         nextZDot = zDot + deltaT * (fz / mass - gravityZ);

         expectedNextState = new DMatrixRMaj(6, 1);
         expectedNextState.set(0, nextX);
         expectedNextState.set(1, nextY);
         expectedNextState.set(2, nextZ);
         expectedNextState.set(3, nextXDot);
         expectedNextState.set(4, nextYDot);
         expectedNextState.set(5, nextZDot);

         MatrixTestTools.assertMatrixEquals(expectedNextState, nextState, 1e-10);
      }
   }

   @Test
   public void testDynamicsStateGradient()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      DMatrixRMaj dynamicsStateGradient = new DMatrixRMaj(6, 6);

      dynamics.getDynamicsStateGradient(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, dynamicsStateGradient);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      double f00 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
      double f02 = -0.5 * deltaT * deltaT * (x - pX) * fz * mass / (Math.pow(mass * z, 2.0));
      double f03 = deltaT;
      double f11 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
      double f12 = -0.5 * deltaT * deltaT * (y - pY) * fz * mass / (Math.pow(mass * z, 2.0));
      double f14 = deltaT;
      double f22 = 1;
      double f25 = deltaT;
      double f30 = deltaT * fz / (mass * z);
      double f32 = - deltaT * (x - pX) * mass * fz / Math.pow(mass * z, 2.0);
      double f33 = 1;
      double f41 = deltaT * fz / (mass * z);
      double f42 = - deltaT * (y - pY) * mass * fz / Math.pow(mass * z, 2.0);
      double f44 = 1;
      double f55 = 1;

      DMatrixRMaj expectedDynamicsStateGradient = new DMatrixRMaj(6, 6);
      expectedDynamicsStateGradient.set(0, 0, f00);
      expectedDynamicsStateGradient.set(0, 2, f02);
      expectedDynamicsStateGradient.set(0, 3, f03);
      expectedDynamicsStateGradient.set(1, 1, f11);
      expectedDynamicsStateGradient.set(1, 2, f12);
      expectedDynamicsStateGradient.set(1, 4, f14);
      expectedDynamicsStateGradient.set(2, 2, f22);
      expectedDynamicsStateGradient.set(2, 5, f25);
      expectedDynamicsStateGradient.set(3, 0, f30);
      expectedDynamicsStateGradient.set(3, 2, f32);
      expectedDynamicsStateGradient.set(3, 3, f33);
      expectedDynamicsStateGradient.set(4, 1, f41);
      expectedDynamicsStateGradient.set(4, 2, f42);
      expectedDynamicsStateGradient.set(4, 4, f44);
      expectedDynamicsStateGradient.set(5, 5, f55);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateGradient, dynamicsStateGradient, 1e-10);

      DMatrixRMaj finiteDifferenceExpectedGradient = new DMatrixRMaj(6, 6);

      double size = 1e-4;
      DMatrixRMaj nextState = new DMatrixRMaj(6, 1);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, nextState);

      DMatrixRMaj variedNextState = new DMatrixRMaj(6, 1);
      DMatrixRMaj variedState = new DMatrixRMaj(currentState);
      variedState.add(0, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      DMatrixRMaj tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 0, tempMatrix, 0, 0, 6, 1, 1.0);

      variedState.add(0, 0, -size);
      variedState.add(1, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 1, tempMatrix, 0, 0, 6, 1, 1.0);

      variedState.add(1, 0, -size);
      variedState.add(2, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 2, tempMatrix, 0, 0, 6, 1, 1.0);

      variedState.add(2, 0, -size);
      variedState.add(3, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 3, tempMatrix, 0, 0, 6, 1, 1.0);

      variedState.add(3, 0, -size);
      variedState.add(4, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 4, tempMatrix, 0, 0, 6, 1, 1.0);

      variedState.add(4, 0, -size);
      variedState.add(5, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, variedState, currentControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 5, tempMatrix, 0, 0, 6, 1, 1.0);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateGradient, finiteDifferenceExpectedGradient, 1e-5);

      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         dynamicsStateGradient = new DMatrixRMaj(6, 6);

         dynamics.getDynamicsStateGradient(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, dynamicsStateGradient);

         assertEquals(6, dynamics.getStateVectorSize());
         assertEquals(3, dynamics.getControlVectorSize());

         f00 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
         f02 = -0.5 * deltaT * deltaT * (x - pX) * fz * mass / (Math.pow(mass * z, 2.0));
         f03 = deltaT;
         f11 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
         f12 = -0.5 * deltaT * deltaT * (y - pY) * fz * mass / (Math.pow(mass * z, 2.0));
         f14 = deltaT;
         f22 = 1;
         f25 = deltaT;
         f30 = deltaT * fz / (mass * z);
         f32 = - deltaT * (x - pX) * mass * fz / Math.pow(mass * z, 2.0);
         f33 = 1;
         f41 = deltaT * fz / (mass * z);
         f42 = - deltaT * (y - pY) * mass * fz / Math.pow(mass * z, 2.0);
         f44 = 1;
         f55 = 1;

         expectedDynamicsStateGradient = new DMatrixRMaj(6, 6);
         expectedDynamicsStateGradient.set(0, 0, f00);
         expectedDynamicsStateGradient.set(0, 2, f02);
         expectedDynamicsStateGradient.set(0, 3, f03);
         expectedDynamicsStateGradient.set(1, 1, f11);
         expectedDynamicsStateGradient.set(1, 2, f12);
         expectedDynamicsStateGradient.set(1, 4, f14);
         expectedDynamicsStateGradient.set(2, 2, f22);
         expectedDynamicsStateGradient.set(2, 5, f25);
         expectedDynamicsStateGradient.set(3, 0, f30);
         expectedDynamicsStateGradient.set(3, 2, f32);
         expectedDynamicsStateGradient.set(3, 3, f33);
         expectedDynamicsStateGradient.set(4, 1, f41);
         expectedDynamicsStateGradient.set(4, 2, f42);
         expectedDynamicsStateGradient.set(4, 4, f44);
         expectedDynamicsStateGradient.set(5, 5, f55);
      }
   }

   @Test
   public void testDynamicsControlGradient()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      DMatrixRMaj dynamicsControlGradient = new DMatrixRMaj(6, 3);

      dynamics.getDynamicsControlGradient(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, dynamicsControlGradient);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      double f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
      double f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
      double f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
      double f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
      double f22 = 0.5 * deltaT * deltaT / mass;
      double f30 = -deltaT * fz / (mass * z);
      double f32 = deltaT * (x - pX) / (mass * z);
      double f41 = -deltaT * fz / (mass * z);
      double f42 = deltaT * (y - pY) / (mass * z);
      double f52 = deltaT / mass;

      DMatrixRMaj expectedDynamicsControlGradient = new DMatrixRMaj(6, 3);
      expectedDynamicsControlGradient.set(0, 0, f00);
      expectedDynamicsControlGradient.set(0, 2, f02);
      expectedDynamicsControlGradient.set(1, 1, f11);
      expectedDynamicsControlGradient.set(1, 2, f12);
      expectedDynamicsControlGradient.set(2, 2, f22);
      expectedDynamicsControlGradient.set(3, 0, f30);
      expectedDynamicsControlGradient.set(3, 2, f32);
      expectedDynamicsControlGradient.set(4, 1, f41);
      expectedDynamicsControlGradient.set(4, 2, f42);
      expectedDynamicsControlGradient.set(5, 2, f52);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlGradient, dynamicsControlGradient, 1e-10);


      DMatrixRMaj finiteDifferenceExpectedGradient = new DMatrixRMaj(6, 3);

      double size = 1e-4;
      DMatrixRMaj nextState = new DMatrixRMaj(6, 1);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, nextState);

      DMatrixRMaj variedNextState = new DMatrixRMaj(6, 1);
      DMatrixRMaj variedControl = new DMatrixRMaj(currentControl);
      variedControl.add(0, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, variedControl, constants, variedNextState);

      DMatrixRMaj tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 0, tempMatrix, 0, 0, 6, 1, 1.0);

      variedControl.add(0, 0, -size);
      variedControl.add(1, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, variedControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 1, tempMatrix, 0, 0, 6, 1, 1.0);

      variedControl.add(1, 0, -size);
      variedControl.add(2, 0, size);
      dynamics.getNextState(DefaultDiscreteState.DEFAULT, currentState, variedControl, constants, variedNextState);

      tempMatrix = new DMatrixRMaj(variedNextState);
      CommonOps_DDRM.subtractEquals(tempMatrix, nextState);
      CommonOps_DDRM.scale(1.0 / size, tempMatrix);
      MatrixTools.setMatrixBlock(finiteDifferenceExpectedGradient, 0, 2, tempMatrix, 0, 0, 6, 1, 1.0);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlGradient, finiteDifferenceExpectedGradient, 1e-5);

      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         dynamicsControlGradient = new DMatrixRMaj(6, 3);

         dynamics.getDynamicsControlGradient(DefaultDiscreteState.DEFAULT, currentState, currentControl, constants, dynamicsControlGradient);

         assertEquals(6, dynamics.getStateVectorSize());
         assertEquals(3, dynamics.getControlVectorSize());

         f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
         f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
         f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
         f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
         f22 = 0.5 * deltaT * deltaT / mass;
         f30 = -deltaT * fz / (mass * z);
         f32 = deltaT * (x - pX) / (mass * z);
         f41 = -deltaT * fz / (mass * z);
         f42 = deltaT * (y - pY) / (mass * z);
         f52 = deltaT / mass;

         expectedDynamicsControlGradient = new DMatrixRMaj(6, 3);
         expectedDynamicsControlGradient.set(0, 0, f00);
         expectedDynamicsControlGradient.set(0, 2, f02);
         expectedDynamicsControlGradient.set(1, 1, f11);
         expectedDynamicsControlGradient.set(1, 2, f12);
         expectedDynamicsControlGradient.set(2, 2, f22);
         expectedDynamicsControlGradient.set(3, 0, f30);
         expectedDynamicsControlGradient.set(3, 2, f32);
         expectedDynamicsControlGradient.set(4, 1, f41);
         expectedDynamicsControlGradient.set(4, 2, f42);
         expectedDynamicsControlGradient.set(5, 2, f52);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlGradient, dynamicsControlGradient, 1e-10);
      }
   }


   @Test
   public void testDynamicsStateHessian()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      //double f00 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
      //double f02 = -0.5 * deltaT * deltaT * (x - pX) * fz * mass / (Math.pow(mass * z, 2.0));
      //double f11 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
      //double f12 = -0.5 * deltaT * deltaT * (y - pY) * fz * mass / (Math.pow(mass * z, 2.0));
      //double f30 = deltaT * fz / (mass * z);
      //double f32 = - deltaT * (x - pX) * mass * fz / Math.pow(mass * z, 2.0);
      //double f41 = deltaT * fz / (mass * z);
      //double f42 = - deltaT * (y - pY) * mass * fz / Math.pow(mass * z, 2.0);

      DMatrixRMaj dynamicsStateHessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj expectedDynamicsStateHessian = new DMatrixRMaj(6, 6);

      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsStateHessian);

      double f02 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      double f32 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
      expectedDynamicsStateHessian.set(0, 2, f02);
      expectedDynamicsStateHessian.set(3, 2, f32);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);



      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsStateHessian);

      double f12 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      double f42 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
      expectedDynamicsStateHessian.zero();
      expectedDynamicsStateHessian.set(1, 2, f12);
      expectedDynamicsStateHessian.set(4, 2, f42);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);



      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsStateHessian);

      double f00 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f02 = 0.5 * deltaT * deltaT * (x - pX) * fz * mass * 2 * mass * mass * z / (Math.pow(mass * z, 4.0));
      double f11 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f12 = 0.5 * deltaT * deltaT * (y - pY) * fz * mass * 2 * mass * mass * z / (Math.pow(mass * z, 4.0));
      double f30 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f32 = deltaT * (x - pX) * mass * fz * 2 * mass * mass * z / Math.pow(mass * z, 4.0);
      double f41 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f42 = deltaT * (y - pY) * mass * fz * 2 * mass * mass * z / Math.pow(mass * z, 4.0);

      expectedDynamicsStateHessian.zero();
      expectedDynamicsStateHessian.set(0, 0, f00);
      expectedDynamicsStateHessian.set(0, 2, f02);
      expectedDynamicsStateHessian.set(1, 1, f11);
      expectedDynamicsStateHessian.set(1, 2, f12);
      expectedDynamicsStateHessian.set(3, 0, f30);
      expectedDynamicsStateHessian.set(3, 2, f32);
      expectedDynamicsStateHessian.set(4, 1, f41);
      expectedDynamicsStateHessian.set(4, 2, f42);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 3, currentState, currentControl, constants, dynamicsStateHessian);
      expectedDynamicsStateHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 4, currentState, currentControl, constants, dynamicsStateHessian);
      expectedDynamicsStateHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

      dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 5, currentState, currentControl, constants, dynamicsStateHessian);
      expectedDynamicsStateHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);


      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         //double f00 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
         //double f02 = -0.5 * deltaT * deltaT * (x - pX) * fz * mass / (Math.pow(mass * z, 2.0));
         //double f11 = (1 + 0.5 * deltaT * deltaT * fz / (mass * z));
         //double f12 = -0.5 * deltaT * deltaT * (y - pY) * fz * mass / (Math.pow(mass * z, 2.0));
         //double f30 = deltaT * fz / (mass * z);
         //double f32 = - deltaT * (x - pX) * mass * fz / Math.pow(mass * z, 2.0);
         //double f41 = deltaT * fz / (mass * z);
         //double f42 = - deltaT * (y - pY) * mass * fz / Math.pow(mass * z, 2.0);

         dynamicsStateHessian = new DMatrixRMaj(6, 6);
         expectedDynamicsStateHessian = new DMatrixRMaj(6, 6);

         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsStateHessian);

         f02 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f32 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
         expectedDynamicsStateHessian.set(0, 2, f02);
         expectedDynamicsStateHessian.set(3, 2, f32);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);



         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsStateHessian);

         f12 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f42 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
         expectedDynamicsStateHessian.zero();
         expectedDynamicsStateHessian.set(1, 2, f12);
         expectedDynamicsStateHessian.set(4, 2, f42);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);



         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsStateHessian);

         f00 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f02 = 0.5 * deltaT * deltaT * (x - pX) * fz * mass * 2 * mass * mass * z / (Math.pow(mass * z, 4.0));
         f11 = -0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f12 = 0.5 * deltaT * deltaT * (y - pY) * fz * mass * 2 * mass * mass * z / (Math.pow(mass * z, 4.0));
         f30 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f32 = deltaT * (x - pX) * mass * fz * 2 * mass * mass * z / Math.pow(mass * z, 4.0);
         f41 = -deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f42 = deltaT * (y - pY) * mass * fz * 2 * mass * mass * z / Math.pow(mass * z, 4.0);

         expectedDynamicsStateHessian.zero();
         expectedDynamicsStateHessian.set(0, 0, f00);
         expectedDynamicsStateHessian.set(0, 2, f02);
         expectedDynamicsStateHessian.set(1, 1, f11);
         expectedDynamicsStateHessian.set(1, 2, f12);
         expectedDynamicsStateHessian.set(3, 0, f30);
         expectedDynamicsStateHessian.set(3, 2, f32);
         expectedDynamicsStateHessian.set(4, 1, f41);
         expectedDynamicsStateHessian.set(4, 2, f42);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 3, currentState, currentControl, constants, dynamicsStateHessian);
         expectedDynamicsStateHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 4, currentState, currentControl, constants, dynamicsStateHessian);
         expectedDynamicsStateHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);

         dynamics.getDynamicsStateHessian(DefaultDiscreteState.DEFAULT, 5, currentState, currentControl, constants, dynamicsStateHessian);
         expectedDynamicsStateHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsStateHessian, dynamicsStateHessian, 1e-10);
      }
   }

   @Test
   public void testDynamicsControlHessian()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      DMatrixRMaj dynamicsControlHessian = new DMatrixRMaj(6, 3);

      dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsControlHessian);

      //double f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
      //double f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
      //double f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
      //double f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
      //double f22 = 0.5 * deltaT * deltaT / mass;
      //double f30 = -deltaT * fz / (mass * z);
      //double f32 = deltaT * (x - pX) / (mass * z);
      //double f41 = -deltaT * fz / (mass * z);
      //double f42 = deltaT * (y - pY) / (mass * z);
      //double f52 = deltaT / mass;

      double f02 = -0.5 * deltaT * deltaT / (mass * z);
      double f32 = -deltaT / (mass * z);

      DMatrixRMaj expectedDynamicsControlHessian = new DMatrixRMaj(6, 3);
      expectedDynamicsControlHessian.set(0, 2, f02);
      expectedDynamicsControlHessian.set(3, 2, f32);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsControlHessian);

      double f12 = -0.5 * deltaT * deltaT / (mass * z);
      double f42 = -deltaT / (mass * z);

      expectedDynamicsControlHessian.zero();
      expectedDynamicsControlHessian.set(1, 2, f12);
      expectedDynamicsControlHessian.set(4, 2, f42);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsControlHessian);

      double f00 = -0.5 * deltaT * deltaT / (mass * z);
      double f11 = -0.5 * deltaT * deltaT / (mass * z);
      double f30 = -deltaT / (mass * z);
      double f41 = -deltaT / (mass * z);

      expectedDynamicsControlHessian.zero();
      expectedDynamicsControlHessian.set(0, 0, f00);
      expectedDynamicsControlHessian.set(1, 1, f11);
      expectedDynamicsControlHessian.set(3, 0, f30);
      expectedDynamicsControlHessian.set(4, 1, f41);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         dynamicsControlHessian = new DMatrixRMaj(6, 3);

         dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsControlHessian);

         //double f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
         //double f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
         //double f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
         //double f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
         //double f22 = 0.5 * deltaT * deltaT / mass;
         //double f30 = -deltaT * fz / (mass * z);
         //double f32 = deltaT * (x - pX) / (mass * z);
         //double f41 = -deltaT * fz / (mass * z);
         //double f42 = deltaT * (y - pY) / (mass * z);
         //double f52 = deltaT / mass;

         f02 = -0.5 * deltaT * deltaT / (mass * z);
         f32 = -deltaT / (mass * z);

         expectedDynamicsControlHessian = new DMatrixRMaj(6, 3);
         expectedDynamicsControlHessian.set(0, 2, f02);
         expectedDynamicsControlHessian.set(3, 2, f32);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

         dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsControlHessian);

         f12 = -0.5 * deltaT * deltaT / (mass * z);
         f42 = -deltaT / (mass * z);

         expectedDynamicsControlHessian.zero();
         expectedDynamicsControlHessian.set(1, 2, f12);
         expectedDynamicsControlHessian.set(4, 2, f42);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

         dynamics.getDynamicsControlHessian(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsControlHessian);

         f00 = -0.5 * deltaT * deltaT / (mass * z);
         f11 = -0.5 * deltaT * deltaT / (mass * z);
         f30 = -deltaT / (mass * z);
         f41 = -deltaT / (mass * z);

         expectedDynamicsControlHessian.zero();
         expectedDynamicsControlHessian.set(0, 0, f00);
         expectedDynamicsControlHessian.set(1, 1, f11);
         expectedDynamicsControlHessian.set(3, 0, f30);
         expectedDynamicsControlHessian.set(4, 1, f41);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);
      }
   }

   @Test
   public void testDynamicsStateGradientOfControlGradient()
   {
      double deltaT = 0.01;
      double mass = 10.0;
      double gravityZ = 9.81;

      double x = 5.0;
      double y = 1.0;
      double z = 1.2;
      double xDot = 1.5;
      double yDot = -2.5;
      double zDot = 0.5;

      double pX = 4.5;
      double pY = 1.5;
      double fz = 80;

      LIPMDynamics dynamics = new LIPMDynamics(deltaT, mass, gravityZ);

      DMatrixRMaj currentState = new DMatrixRMaj(6, 1);
      currentState.set(0, x);
      currentState.set(1, y);
      currentState.set(2, z);
      currentState.set(3, xDot);
      currentState.set(4, yDot);
      currentState.set(5, zDot);

      DMatrixRMaj currentControl = new DMatrixRMaj(3, 1);
      currentControl.set(0, pX);
      currentControl.set(1, pY);
      currentControl.set(2, fz);

      DMatrixRMaj constants = new DMatrixRMaj(0, 1);

      assertEquals(6, dynamics.getStateVectorSize());
      assertEquals(3, dynamics.getControlVectorSize());

      DMatrixRMaj dynamicsControlHessian = new DMatrixRMaj(6, 3);

      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsControlHessian);

      //double f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
      //double f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
      //double f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
      //double f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
      //double f22 = 0.5 * deltaT * deltaT / mass;
      //double f30 = -deltaT * fz / (mass * z);
      //double f32 = deltaT * (x - pX) / (mass * z);
      //double f41 = -deltaT * fz / (mass * z);
      //double f42 = deltaT * (y - pY) / (mass * z);
      //double f52 = deltaT / mass;

      double f02 = 0.5 * deltaT * deltaT / (mass * z);
      double f32 = deltaT / (mass * z);

      DMatrixRMaj expectedDynamicsControlHessian = new DMatrixRMaj(6, 3);
      expectedDynamicsControlHessian.set(0, 2, f02);
      expectedDynamicsControlHessian.set(3, 2, f32);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);



      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsControlHessian);

      double f12 = 0.5 * deltaT * deltaT / (mass * z);
      double f42 = deltaT / (mass * z);

      expectedDynamicsControlHessian.zero();
      expectedDynamicsControlHessian.set(1, 2, f12);
      expectedDynamicsControlHessian.set(4, 2, f42);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);



      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsControlHessian);

      double f00 = 0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f02 = -0.5 * deltaT * deltaT * (x - pX) * mass / Math.pow(mass * z, 2.0);
      double f11 = 0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f12 = -0.5 * deltaT * deltaT * (y - pY) * mass / Math.pow(mass * z, 2.0);
      double f30 = deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f32 = -deltaT * (x - pX) * mass / Math.pow(mass * z, 2.0);
      double f41 = deltaT * fz * mass / Math.pow(mass * z, 2.0);
      f42 = -deltaT * (y - pY) * mass / Math.pow(mass * z, 2.0);

      expectedDynamicsControlHessian.zero();
      expectedDynamicsControlHessian.set(0, 0, f00);
      expectedDynamicsControlHessian.set(0, 2, f02);
      expectedDynamicsControlHessian.set(1, 1, f11);
      expectedDynamicsControlHessian.set(1, 2, f12);
      expectedDynamicsControlHessian.set(3, 0, f30);
      expectedDynamicsControlHessian.set(3, 2, f32);
      expectedDynamicsControlHessian.set(4, 1, f41);
      expectedDynamicsControlHessian.set(4, 2, f42);

      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);





      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 3, currentState, currentControl, constants, dynamicsControlHessian);
      expectedDynamicsControlHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 4, currentState, currentControl, constants, dynamicsControlHessian);
      expectedDynamicsControlHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 5, currentState, currentControl, constants, dynamicsControlHessian);
      expectedDynamicsControlHessian.zero();
      MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

      assertEquals(x, currentState.get(0), 1e-10);
      assertEquals(y, currentState.get(1), 1e-10);
      assertEquals(z, currentState.get(2), 1e-10);
      assertEquals(xDot, currentState.get(3), 1e-10);
      assertEquals(yDot, currentState.get(4), 1e-10);
      assertEquals(zDot, currentState.get(5), 1e-10);

      assertEquals(pX, currentControl.get(0), 1e-10);
      assertEquals(pY, currentControl.get(1), 1e-10);
      assertEquals(fz, currentControl.get(2), 1e-10);

      Random random = new Random(10L);
      for (int i = 0; i < 100; i++)
      {
         x = RandomNumbers.nextDouble(random, 10.0);
         y = RandomNumbers.nextDouble(random, 10.0);
         z = RandomNumbers.nextDouble(random, 0, 5);
         xDot = RandomNumbers.nextDouble(random, 0, 10);
         yDot = RandomNumbers.nextDouble(random, 0, 10);
         zDot = RandomNumbers.nextDouble(random, 0, 10);

         pX = RandomNumbers.nextDouble(random, 10.0);
         pY = RandomNumbers.nextDouble(random, 10.0);
         fz = RandomNumbers.nextDouble(random, 0, 1000);

         currentState = new DMatrixRMaj(6, 1);
         currentState.set(0, x);
         currentState.set(1, y);
         currentState.set(2, z);
         currentState.set(3, xDot);
         currentState.set(4, yDot);
         currentState.set(5, zDot);

         currentControl = new DMatrixRMaj(3, 1);
         currentControl.set(0, pX);
         currentControl.set(1, pY);
         currentControl.set(2, fz);

         constants = new DMatrixRMaj(0, 1);

         dynamicsControlHessian = new DMatrixRMaj(6, 3);

         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 0, currentState, currentControl, constants, dynamicsControlHessian);

         //double f00 = -0.5 * deltaT * deltaT * fz / (mass * z);
         //double f02 = 0.5 * deltaT * deltaT * (x - pX) / (mass * z);
         //double f11 = -0.5 * deltaT * deltaT * fz / (mass * z);
         //double f12 = 0.5 * deltaT * deltaT * (y - pY) / (mass * z);
         //double f22 = 0.5 * deltaT * deltaT / mass;
         //double f30 = -deltaT * fz / (mass * z);
         //double f32 = deltaT * (x - pX) / (mass * z);
         //double f41 = -deltaT * fz / (mass * z);
         //double f42 = deltaT * (y - pY) / (mass * z);
         //double f52 = deltaT / mass;

         f02 = 0.5 * deltaT * deltaT / (mass * z);
         f32 = deltaT / (mass * z);

         expectedDynamicsControlHessian = new DMatrixRMaj(6, 3);
         expectedDynamicsControlHessian.set(0, 2, f02);
         expectedDynamicsControlHessian.set(3, 2, f32);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);



         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 1, currentState, currentControl, constants, dynamicsControlHessian);

         f12 = 0.5 * deltaT * deltaT / (mass * z);
         f42 = deltaT / (mass * z);

         expectedDynamicsControlHessian.zero();
         expectedDynamicsControlHessian.set(1, 2, f12);
         expectedDynamicsControlHessian.set(4, 2, f42);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);



         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 2, currentState, currentControl, constants, dynamicsControlHessian);

         f00 = 0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f02 = -0.5 * deltaT * deltaT * (x - pX) * mass / Math.pow(mass * z, 2.0);
         f11 = 0.5 * deltaT * deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f12 = -0.5 * deltaT * deltaT * (y - pY) * mass / Math.pow(mass * z, 2.0);
         f30 = deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f32 = -deltaT * (x - pX) * mass / Math.pow(mass * z, 2.0);
         f41 = deltaT * fz * mass / Math.pow(mass * z, 2.0);
         f42 = -deltaT * (y - pY) * mass / Math.pow(mass * z, 2.0);

         expectedDynamicsControlHessian.zero();
         expectedDynamicsControlHessian.set(0, 0, f00);
         expectedDynamicsControlHessian.set(0, 2, f02);
         expectedDynamicsControlHessian.set(1, 1, f11);
         expectedDynamicsControlHessian.set(1, 2, f12);
         expectedDynamicsControlHessian.set(3, 0, f30);
         expectedDynamicsControlHessian.set(3, 2, f32);
         expectedDynamicsControlHessian.set(4, 1, f41);
         expectedDynamicsControlHessian.set(4, 2, f42);

         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);





         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 3, currentState, currentControl, constants, dynamicsControlHessian);
         expectedDynamicsControlHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 4, currentState, currentControl, constants, dynamicsControlHessian);
         expectedDynamicsControlHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);

         dynamics.getDynamicsStateGradientOfControlGradient(DefaultDiscreteState.DEFAULT, 5, currentState, currentControl, constants, dynamicsControlHessian);
         expectedDynamicsControlHessian.zero();
         MatrixTestTools.assertMatrixEquals(expectedDynamicsControlHessian, dynamicsControlHessian, 1e-10);
      }
   }
}
