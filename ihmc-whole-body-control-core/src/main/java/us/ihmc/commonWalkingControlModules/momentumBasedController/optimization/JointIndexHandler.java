package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.matrixlib.NativeMatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class JointIndexHandler
{
   private final TIntArrayList indicesIntoCompactBlock = new TIntArrayList();
   private final LinkedHashMap<JointReadOnly, int[]> columnsForJoints = new LinkedHashMap<>();

   private final int numberOfDoFs;
   private final JointBasics[] indexedJoints;
   private final OneDoFJointBasics[] indexedOneDoFJoints;

   public JointIndexHandler(JointBasics[] jointsToIndex)
   {
      indexedJoints = jointsToIndex;
      indexedOneDoFJoints = MultiBodySystemTools.filterJoints(indexedJoints, OneDoFJointBasics.class);

      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToIndex);
      populateColumnIndices();
   }

   public JointIndexHandler(List<? extends JointBasics> jointsToIndex)
   {
      indexedJoints = new JointBasics[jointsToIndex.size()];
      jointsToIndex.toArray(indexedJoints);
      indexedOneDoFJoints = MultiBodySystemTools.filterJoints(indexedJoints, OneDoFJointBasics.class);

      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsToIndex);
      populateColumnIndices();
   }

   private void populateColumnIndices()
   {
      for (JointBasics joint : indexedJoints)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         computeIndexForJoint(indexedJoints, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();

         columnsForJoints.put(joint, indices);
      }
   }

   public boolean compactBlockToFullBlock(JointReadOnly[] joints, DMatrixRMaj compactMatrix, DMatrixRMaj fullMatrix)
   {
      fullMatrix.zero();

      for (JointReadOnly joint : joints)
      {
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return false;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps_DDRM.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }

      return true;
   }
   
   public boolean compactBlockToFullBlock(JointReadOnly[] joints, DMatrixRMaj compactMatrix, NativeMatrix fullMatrix)
   {
      fullMatrix.zero();
      
      for (JointReadOnly joint : joints)
      {
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);
         
         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return false;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
           
            fullMatrix.insert(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, 0, fullBlockIndex);
         }
      }
      
      return true;
   }

   public boolean compactBlockToFullBlock(List<? extends JointReadOnly> joints, DMatrixRMaj compactMatrix, DMatrixRMaj fullMatrix)
   {
      fullMatrix.zero();

      for (int index = 0; index < joints.size(); index++)
      {
         JointReadOnly joint = joints.get(index);
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            return false;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps_DDRM.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }

      return true;
   }

   public void compactBlockToFullBlockIgnoreUnindexedJoints(List<? extends JointReadOnly> joints, DMatrixRMaj compactMatrix, DMatrixRMaj fullMatrix)
   {
      fullMatrix.reshape(compactMatrix.getNumRows(), fullMatrix.getNumCols());
      fullMatrix.zero();

      for (int index = 0; index < joints.size(); index++)
      {
         JointReadOnly joint = joints.get(index);
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            continue;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps_DDRM.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public void compactBlockToFullBlockIgnoreUnindexedJoints(List<? extends JointReadOnly> joints, NativeMatrix compactMatrix, NativeMatrix fullMatrix)
   {
      fullMatrix.reshape(compactMatrix.getNumRows(), fullMatrix.getNumCols());
      fullMatrix.zero();

      int rows = compactMatrix.getNumRows();

      for (int index = 0; index < joints.size(); index++)
      {
         JointReadOnly joint = joints.get(index);
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            continue;

         NativeMatrixTools.extractColumns(compactMatrix, indicesIntoCompactBlock, fullMatrix, indicesIntoFullBlock);
      }
   }

   public void compactBlockToFullBlockIgnoreUnindexedJoints(JointBasics[] joints, DMatrixRMaj compactMatrix, DMatrixRMaj fullMatrix)
   {
      fullMatrix.reshape(compactMatrix.getNumRows(), fullMatrix.getNumCols());
      fullMatrix.zero();

      for (int index = 0; index < joints.length; index++)
      {
         JointBasics joint = joints[index];
         indicesIntoCompactBlock.reset();
         computeIndexForJoint(joints, indicesIntoCompactBlock, joint);
         int[] indicesIntoFullBlock = columnsForJoints.get(joint);

         if (indicesIntoFullBlock == null) // don't do anything for joints that are not in the list
            continue;

         for (int i = 0; i < indicesIntoCompactBlock.size(); i++)
         {
            int compactBlockIndex = indicesIntoCompactBlock.get(i);
            int fullBlockIndex = indicesIntoFullBlock[i];
            CommonOps_DDRM.extract(compactMatrix, 0, compactMatrix.getNumRows(), compactBlockIndex, compactBlockIndex + 1, fullMatrix, 0, fullBlockIndex);
         }
      }
   }

   public static void computeIndicesForJoint(JointReadOnly[] jointsInOrder, TIntArrayList listToPackIndices, JointReadOnly... jointsToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         for (int j = 0; j < jointsToComputeIndicesFor.length; j++)
         {
            if (jointsInOrder[i] == jointsToComputeIndicesFor[j])
            {
               for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
               {
                  listToPackIndices.add(k);
               }
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static void computeIndexForJoint(List<? extends JointReadOnly> jointsInOrder, TIntArrayList listToPackIndices, JointReadOnly jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.size(); i++)
      {
         JointReadOnly joint = jointsInOrder.get(i);
         int nDegreesOfFreedom = joint.getDegreesOfFreedom();

         if (joint == jointToComputeIndicesFor)
         {
            for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
            {
               listToPackIndices.add(k);
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static void computeIndexForJoint(JointReadOnly[] jointsInOrder, TIntArrayList listToPackIndices, JointReadOnly jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         if (jointsInOrder[i] == jointToComputeIndicesFor)
         {
            for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
            {
               listToPackIndices.add(k);
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public JointBasics[] getIndexedJoints()
   {
      return indexedJoints;
   }

   public OneDoFJointBasics[] getIndexedOneDoFJoints()
   {
      return indexedOneDoFJoints;
   }

   public boolean isJointIndexed(JointBasics joint)
   {
      return columnsForJoints.containsKey(joint);
   }

   public boolean areJointsIndexed(JointBasics[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (!isJointIndexed(joints[i]))
            return false;
      }
      return true;
   }

   public int getOneDoFJointIndex(OneDoFJointReadOnly joint)
   {
      int[] jointIndices = columnsForJoints.get(joint);
      if (jointIndices == null)
         return -1;
      else
         return jointIndices[0];
   }

   public int[] getJointIndices(JointReadOnly joint)
   {
      return columnsForJoints.get(joint);
   }

   public int getNumberOfDoFs()
   {
      return numberOfDoFs;
   }
}
