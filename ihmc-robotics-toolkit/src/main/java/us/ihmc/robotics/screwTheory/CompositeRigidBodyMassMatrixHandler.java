package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class CompositeRigidBodyMassMatrixHandler
{
   private final DenseMatrix64F columnReducedMassMatrix;
   private final DenseMatrix64F massMatrix;

   private final int degreesOfFreedom;

   private final Map<JointBasics, int[]> indicesForJoints = new LinkedHashMap<>();
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;

   public CompositeRigidBodyMassMatrixHandler(RigidBody rootBody, ArrayList<JointBasics> jointsToIgnore)
   {
      this.massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody, jointsToIgnore);

      JointBasics[] jointsInOrder = massMatrixCalculator.getJointsInOrder();
      for (JointBasics joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();
         indicesForJoints.put(joint, indices);
      }

      degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      columnReducedMassMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);
      massMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);
   }

   public void compute()
   {
      massMatrixCalculator.compute();
   }

   public DenseMatrix64F getMassMatrix(JointBasics[] jointsToConsider)
   {
      int reducedDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsToConsider);
      massMatrix.reshape(reducedDegreesOfFreedom, reducedDegreesOfFreedom);
      massMatrix.zero();

      getMassMatrix(jointsToConsider, massMatrix);

      return massMatrix;
   }

   public void getMassMatrix(JointBasics[] jointsToConsider, DenseMatrix64F massMatrixToPack)
   {
      int reducedDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsToConsider);
      columnReducedMassMatrix.reshape(degreesOfFreedom, reducedDegreesOfFreedom);
      columnReducedMassMatrix.zero();
      massMatrixToPack.zero();

      int startColumn = 0;
      for (JointBasics joint : jointsToConsider)
      {
         int[] columnsForJoint = indicesForJoints.get(joint);
         MatrixTools.extractColumns(massMatrixCalculator.getMassMatrix(), columnsForJoint, columnReducedMassMatrix, startColumn);
         startColumn += columnsForJoint.length;
      }

      int startRow = 0;
      for (JointBasics joint : jointsToConsider)
      {
         int[] rowsForJoint = indicesForJoints.get(joint);
         MatrixTools.extractRows(columnReducedMassMatrix, rowsForJoint, massMatrixToPack, startRow);
         startRow += rowsForJoint.length;
      }
   }
}
