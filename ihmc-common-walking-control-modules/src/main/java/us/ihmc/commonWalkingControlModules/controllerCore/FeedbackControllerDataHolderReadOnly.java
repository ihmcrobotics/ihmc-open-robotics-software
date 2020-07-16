package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.data.PositionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.QuaternionData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.data.VectorData3D;
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
   void getCenterOfMassPositionData(List<PositionData3D> positionDataListToPack, Type type);

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
    *                         in {@link Space}.
    */
   void getCenterOfMassVectorData(List<VectorData3D> vectorDataListToPack, Type type, Space space);

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
   void getPositionData(RigidBodyBasics endEffector, List<PositionData3D> positionDataListToPack, Type type);

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
   void getOrientationData(RigidBodyBasics endEffector, List<QuaternionData3D> orientationDataListToPack, Type type);

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
    *                         in {@link Space}.
    */
   void getVectorData(RigidBodyBasics endEffector, List<VectorData3D> vectorDataListToPack, Type type, Space space);

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
