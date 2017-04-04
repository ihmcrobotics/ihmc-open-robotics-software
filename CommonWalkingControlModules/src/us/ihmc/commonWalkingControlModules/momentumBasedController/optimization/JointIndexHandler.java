package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class JointIndexHandler
{
   private final TIntArrayList indicesIntoCompactBlock = new TIntArrayList();
   private final LinkedHashMap<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();

   private final int numberOfDoFs;
   private final InverseDynamicsJoint[] indexedJoints;
   private final OneDoFJoint[] indexedOneDoFJoints;

   public JointIndexHandler(InverseDynamicsJoint[] jointsToIndex)
   {
      indexedJoints = jointsToIndex;
      indexedOneDoFJoints = ScrewTools.filterJoints(indexedJoints, OneDoFJoint.class);

      numberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsToIndex);

      for (InverseDynamicsJoint joint : jointsToIndex)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsToIndex, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         columnsForJoints.put(joint, indices);
      }
   }

   public boolean compactBlockToFullBlock(InverseDynamicsJoint[] joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.length; index++)
      {
         InverseDynamicsJoint joint = joints[index];
         indicesIntoCompactBlock.reset();
         ScrewTools.computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return false;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }

      return true;
   }

   public void compactBlockToFullBlockIgnoreUnindexedJoints(List<InverseDynamicsJoint> joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.size(); index++)
      {
         InverseDynamicsJoint joint = joints.get(index);
         indicesIntoCompactBlock.reset();
         ScrewTools.computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            continue;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public void compactBlockToFullBlockIgnoreUnindexedJoints(InverseDynamicsJoint[] joints, DenseMatrix64F compactMatrix, DenseMatrix64F fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.length; index++)
      {
         InverseDynamicsJoint joint = joints[index];
         indicesIntoCompactBlock.reset();
         ScrewTools.computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            continue;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public InverseDynamicsJoint[] getIndexedJoints()
   {
      return indexedJoints;
   }

   public OneDoFJoint[] getIndexedOneDoFJoints()
   {
      return indexedOneDoFJoints;
   }

   public boolean isJointIndexed(InverseDynamicsJoint joint)
   {
      return columnsForJoints.containsKey(joint);
   }

   public boolean areJointsIndexed(InverseDynamicsJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (!isJointIndexed(joints[i]))
            return false;
      }
      return true;
   }

   public int getOneDoFJointIndex(OneDoFJoint joint)
   {
      int[] jointIndices = columnsForJoints.get(joint);
      if (jointIndices == null)
         return -1;
      else
         return jointIndices[0];
   }

   public int[] getJointIndices(InverseDynamicsJoint joint)
   {
      return columnsForJoints.get(joint);
   }

   public int getNumberOfDoFs()
   {
      return numberOfDoFs;
   }
}
