package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.ScrewTools;

/*
 * Change the strategy to be less expensive in terms of computation.
 * The method compute() should be replaced a method reset() which should flip a boolean somehow linked to each jacobian indicating that they need to be updated.
 * When a getter is called, call GeometricJacobian.compute() only if needed.
 * That should allow the user to create as many jacobian as needed and update only the ones that are actually being used.
 */
public class GeometricJacobianHolder
{
   public static final int NULL_JACOBIAN_ID = 0;

   private final TIntObjectHashMap<GeometricJacobian> hashCodeToJacobianMap = new TIntObjectHashMap<GeometricJacobian>();
   private final List<GeometricJacobian> geometricJacobians = new ArrayList<GeometricJacobian>();
   private final JointBasics[] temporaryToStoreJointPath = new JointBasics[30];

   public void compute()
   {
      for (int i = 0; i < geometricJacobians.size(); i++)
      {
         geometricJacobians.get(i).compute();
      }
   }

   /**
    * Find or create a Jacobian and register it in the {@link HighLevelHumanoidControllerToolbox}.
    * It returns an jacobianId with which it is possible to find the Jacobian later with the method getJacobian(int jacobianId).
    * If the array of joints is empty, it returns -1.
    * @param joints
    * @param jacobianFrame
    * @return
    */
   public int getOrCreateGeometricJacobian(JointBasics[] joints, ReferenceFrame jacobianFrame)
   {
      return getOrCreateGeometricJacobian(joints, joints.length, jacobianFrame);
   }

   private int getOrCreateGeometricJacobian(JointBasics[] joints, int numberOfJointsToConsider, ReferenceFrame jacobianFrame)
   {
      if (joints == null || numberOfJointsToConsider == 0)
         return NULL_JACOBIAN_ID;
      
      // The mapping assumes the frame do not change.
      // On top of that, this class makes the different modules use the same instances of each Jacobian, so it would not be good if one module changes the frame of a Jacobian shared with another module. 
      boolean allowChangeFrame = false;

      int jacobianId = ScrewTools.computeGeometricJacobianHashCode(joints, 0, numberOfJointsToConsider - 1, jacobianFrame, allowChangeFrame);
      GeometricJacobian jacobian = getJacobian(jacobianId);

      if (jacobian == null)
      {
         if (joints.length == numberOfJointsToConsider)
         {
            jacobian = new GeometricJacobian(joints, jacobianFrame, allowChangeFrame);
         }
         else
         {
            JointBasics[] jointsForNewJacobian = new JointBasics[numberOfJointsToConsider];
            System.arraycopy(joints, 0, jointsForNewJacobian, 0, numberOfJointsToConsider);
            jacobian = new GeometricJacobian(jointsForNewJacobian, jacobianFrame, allowChangeFrame);
         }
         jacobian.compute(); // Compute in case you need it right away
         geometricJacobians.add(jacobian);
         hashCodeToJacobianMap.put(jacobian.hashCode(), jacobian);
      }

      return jacobian.hashCode();
   }

   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(int jacobianId)
   {
      if (jacobianId == NULL_JACOBIAN_ID)
         return null;
      else
         return hashCodeToJacobianMap.get(jacobianId);
   }
}
