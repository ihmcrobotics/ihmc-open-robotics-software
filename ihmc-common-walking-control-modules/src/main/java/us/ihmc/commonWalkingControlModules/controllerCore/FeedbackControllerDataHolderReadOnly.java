package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.data.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface FeedbackControllerDataHolderReadOnly
{
   /**
   * Retrieves if possible the position data about the center of mass.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    *
    * @param positionDataToPack the {@link FramePoint3DBasics} in which the position data is stored. Modified.
    * @param type whether the current or desired position is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getCenterOfMassPositionData(FramePoint3DBasics positionDataToPack, Type type);

   /**
    * Retrieves if possible the vector data about the center of mass.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    *
    * @param vectorDataToPack the {@link FrameVector3DBasics} in which the vector data is stored. Modified.
    * @param type specifies the data type requested. Look up the options available in {@link Type}.
    * @param space specifies the physical quantity requested. Look up the options available in
    *           {@link Space}.
    * @return whether the data is available or not.
    */
   boolean getCenterOfMassVectorData(FrameVector3DBasics vectorDataToPack, Type type, Space space);

   /**
    * Retrieves if possible the position data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    *
    * @param endEffector the end-effector for which the data is requested.
    * @param positionDataToPack the {@link FramePoint3DBasics} in which the position data is stored. Modified.
    * @param type whether the current or desired position is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getPositionData(RigidBodyBasics endEffector, FramePoint3DBasics positionDataToPack, Type type);

   /**
    * Retrieves if possible the orientation data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    *
    * @param endEffector the end-effector for which the data is requested.
    * @param orientationDataToPack the {@link FrameQuaternionBasics} in which the orientation data is
    *           stored. Modified.
    * @param type whether the current or desired orientation is requested, the other values in
    *           {@link Type} are not applicable.
    * @return whether the data is available or not.
    */
   boolean getOrientationData(RigidBodyBasics endEffector, FrameQuaternionBasics orientationDataToPack, Type type);

   /**
    * Retrieves if possible the vector data about a specific end-effector.
    * <p>
    * If no feedback controller has used the requested data, it will not be available in which case
    * the method returns {@code false}.
    * </p>
    *
    * @param endEffector the end-effector for which the data is requested.
    * @param vectorDataToPack the {@link FrameVector3DBasics} in which the vector data is stored. Modified.
    * @param type specifies the data type requested. Look up the options available in {@link Type}.
    * @param space specifies the physical quantity requested. Look up the options available in
    *           {@link Space}.
    * @return whether the data is available or not.
    */
   boolean getVectorData(RigidBodyBasics endEffector, FrameVector3DBasics vectorDataToPack, Type type, Space space);
}
