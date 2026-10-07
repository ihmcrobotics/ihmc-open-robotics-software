package us.ihmc.externalControl;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.externalControl.library.ExternalControlNativeLibrary;
import us.ihmc.matrixlib.MatrixTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class ExternalControlImplTest
{
   @Test
   public void testSizes()
   {
      ExternalControlNativeLibrary.load();

      Random random = new Random(1738L);
      double stiffness = 5.0;
      double damping = 1.0;
      int joints = 10;
      ExternalControlWrapper.ExternalControlImpl externalControl = new ExternalControlWrapper.ExternalControlImpl(stiffness, damping, joints);

      for (int i = 0; i < 100; i++)
      {
         DMatrixRMaj homeConfiguration = EuclidCoreRandomTools.nextDMatrixRMaj(random, joints, 1);
         boolean success = externalControl.setHomeJointConfiguration(homeConfiguration.data, homeConfiguration.numRows);
         assertTrue(success, "Failed to set home configuration.");

         int controlSize = joints;
         int stateSize = joints * 2 + 13;
         DMatrixRMaj state = new DMatrixRMaj(stateSize, 1);
         DMatrixRMaj control = new DMatrixRMaj(controlSize, 1);
         DMatrixRMaj feet = new DMatrixRMaj(6, 1);
         success = externalControl.updateRobotState(24.0, state.data, state.numRows, control.data, control.numRows, true, true, feet.data, feet.numRows, 0, 0);
         assertTrue(success, "Failed to successfully compute.");

         DMatrixRMaj returnedState = new DMatrixRMaj(stateSize, 1);
         DMatrixRMaj returnedControl = new DMatrixRMaj(controlSize, 1);
         DMatrixRMaj returnedStiffness = new DMatrixRMaj(controlSize, 1);
         DMatrixRMaj returnedDamping = new DMatrixRMaj(controlSize, 1);
         success = externalControl.getSolution(returnedState.data,
                                               returnedState.numRows,
                                               returnedControl.data,
                                               returnedControl.numRows,
                                               returnedStiffness.data,
                                               returnedStiffness.numRows,
                                               returnedDamping.data,
                                               returnedDamping.numRows);
         assertTrue(success, "Failed to get the solution");

         DMatrixRMaj expectedState = new DMatrixRMaj(stateSize, 1);
         DMatrixRMaj expectedControl = new DMatrixRMaj(controlSize, 1);
         DMatrixRMaj expectedStiffness = new DMatrixRMaj(controlSize, 1);
         DMatrixRMaj expectedDamping = new DMatrixRMaj(controlSize, 1);
         CommonOps_DDRM.fill(expectedStiffness, stiffness);
         CommonOps_DDRM.fill(expectedDamping, damping);
         MatrixTools.setMatrixBlock(expectedState, 7, 0, homeConfiguration, 0, 0, joints, 1, 1.0);

         double epsilon = 1e-4;
         EjmlUnitTests.assertEquals(expectedState, returnedState, epsilon);
         EjmlUnitTests.assertEquals(expectedControl, returnedControl, epsilon);
         EjmlUnitTests.assertEquals(expectedStiffness, returnedStiffness, epsilon);
         EjmlUnitTests.assertEquals(expectedDamping, returnedDamping, epsilon);
      }
   }
}
