package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBPoint3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBQuaternion3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface FeedbackControllerDataHolderReadOnly
{
   /**
    * Retrieves if possible the position data about the center of mass.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * <p>
    * If no feedback controller has used the requested data, the given list is cleared.
    * </p>
    * <p>
    * While there is commonly only one feedback controller handling the center of mass, it is possible
    * that multiple controllers are handling it in which case the desired data for each controller is
    * added to the given list.
    * </p>
    *
    * @param positionDataToPack list used to store the position data available. Modified.
    * @param type               whether the current or desired position is requested, the other values
    *                           in {@link Type} are not applicable.
    */
   void getCenterOfMassPositionData(List<FBPoint3D> positionDataListToPack, Type type);

   /**
    * Retrieves if possible the vector data about the center of mass.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the
    * method returns {@code false}.
    * </p>
    * <p>
    * While there is commonly only one feedback controller handling the center of mass, it is possible
    * that multiple controllers are handling it in which case the desired data for each controller is
    * added to the given list.
    * </p>
    *
    * @param vectorDataToPack list used to store the vector data available. Modified.
    * @param type             specifies the data type requested. Look up the options available in
    *                         {@link Type}.
    * @param space            specifies the physical quantity requested. Look up the options available
    *                         in {@link SpaceData3D}.
    */
   void getCenterOfMassVectorData(List<FBVector3D> vectorDataListToPack, Type type, SpaceData3D space);

   /**
    * Retrieves if possible the position data about a specific end-effector.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the
    * method returns {@code false}.
    * </p>
    * <p>
    * While there is commonly only one feedback controller handling each end-effector, it is possible
    * that multiple controllers are handling one in which case the desired data for each controller is
    * added to the given list.
    * </p>
    *
    * @param endEffector        the end-effector for which the data is requested.
    * @param positionDataToPack list used to store the position data available. Modified.
    * @param type               whether the current or desired position is requested, the other values
    *                           in {@link Type} are not applicable.
    */
   void getPositionData(RigidBodyBasics endEffector, List<FBPoint3D> positionDataListToPack, Type type);

   /**
    * Retrieves if possible the orientation data about a specific end-effector.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the
    * method returns {@code false}.
    * </p>
    * <p>
    * While there is commonly only one feedback controller handling each end-effector, it is possible
    * that multiple controllers are handling one in which case the desired data for each controller is
    * added to the given list.
    * </p>
    *
    * @param endEffector           the end-effector for which the data is requested.
    * @param orientationDataToPack list used to store the orientation data available. Modified.
    * @param type                  whether the current or desired orientation is requested, the other
    *                              values in {@link Type} are not applicable.
    */
   void getOrientationData(RigidBodyBasics endEffector, List<FBQuaternion3D> orientationDataListToPack, Type type);

   /**
    * Retrieves if possible the vector data about a specific end-effector.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case the
    * method returns {@code false}.
    * </p>
    * <p>
    * While there is commonly only one feedback controller handling each end-effector, it is possible
    * that multiple controllers are handling one in which case the desired data for each controller is
    * added to the given list.
    * </p>
    *
    * @param endEffector      the end-effector for which the data is requested.
    * @param vectorDataToPack list used to store the vector data available. Modified.
    * @param type             specifies the data type requested. Look up the options available in
    *                         {@link Type}.
    * @param space            specifies the physical quantity requested. Look up the options available
    *                         in {@link SpaceData3D}.
    */
   void getVectorData(RigidBodyBasics endEffector, List<FBVector3D> vectorDataListToPack, Type type, SpaceData3D space);

   /**
    * Retrieves when applicable, the output calculated by the feedback controller feeding the inverse
    * dynamics optimization module.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * 
    * @return the last output from the feedback controllers.
    */
   InverseDynamicsCommandList getLastFeedbackControllerInverseDynamicsOutput();

   /**
    * Retrieves when applicable, the output calculated by the feedback controller feeding the inverse
    * kinematics optimization module.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * 
    * @return the last output from the feedback controllers.
    */
   InverseKinematicsCommandList getLastFeedbackControllerInverseKinematicsOutput();

   /**
    * Retrieves when applicable, the output calculated by the feedback controller feeding the virtual
    * model control optimization module.
    * <p>
    * The data should not be modified, it is expected to be used for read-only purposes.
    * </p>
    * 
    * @return the last output from the feedback controllers.
    */
   VirtualModelControlCommandList getLastFeedbackControllerVirtualModelControlOutput();
}
