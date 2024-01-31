package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CrocoddylStateMessage;
import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class CrocoddylStateCommand implements Command<CrocoddylStateCommand, CrocoddylStateMessage>
{
   private int sequenceId = -1;
   private final Pose3D basePoseInWorld = new Pose3D();
   private final Point3D basePositionInWorld = basePoseInWorld.getPosition();
   private final Quaternion baseOrientationInWorld = basePoseInWorld.getOrientation();
   private final Vector3D baseLinearRateInBaseFrame = new Vector3D();
   private final Vector3D baseAngularRateInBaseFrame = new Vector3D();

   private final DMatrixRMaj jointPositions = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jointVelocities = new DMatrixRMaj(0, 0);

   @Override
   public void clear()
   {
      sequenceId = -1;
      basePositionInWorld.setToNaN();
      baseOrientationInWorld.setToNaN();
      baseLinearRateInBaseFrame.setToNaN();
      baseAngularRateInBaseFrame.setToNaN();

      jointPositions.reshape(0, 0);
      jointVelocities.reshape(0, 0);
      jointPositions.zero();
      jointVelocities.zero();
   }

   @Override
   public void setFromMessage(CrocoddylStateMessage message)
   {
      sequenceId = (int) message.getUniqueId();

      TDoubleArrayList jointStateVector = message.getX();
      // minus 13 to remove the position, orientation (4), and twist.
      int numberOfJoints = (jointStateVector.size() - 13) / 2;
      int dataPointer = 0;
      basePositionInWorld.set(jointStateVector.get(dataPointer++),
                              jointStateVector.get(dataPointer++),
                              jointStateVector.get(dataPointer++));
      baseOrientationInWorld.set(jointStateVector.get(dataPointer++),
                                 jointStateVector.get(dataPointer++),
                                 jointStateVector.get(dataPointer++),
                                 jointStateVector.get(dataPointer++));

      jointPositions.reshape(numberOfJoints, 1);
      jointVelocities.reshape(numberOfJoints, 1);
      for (int i = 0; i < numberOfJoints; i++)
         jointPositions.set(i, 0, jointStateVector.get(dataPointer++));

      baseLinearRateInBaseFrame.set(jointStateVector.get(dataPointer++),
                                    jointStateVector.get(dataPointer++),
                                    jointStateVector.get(dataPointer++));
      baseAngularRateInBaseFrame.set(jointStateVector.get(dataPointer++),
                                    jointStateVector.get(dataPointer++),
                                    jointStateVector.get(dataPointer++));
      for (int i = 0; i < numberOfJoints; i++)
         jointVelocities.set(i, 0, jointStateVector.get(dataPointer++));
   }

   @Override
   public Class<CrocoddylStateMessage> getMessageClass()
   {
      return CrocoddylStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(CrocoddylStateCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.basePositionInWorld.set(other.basePositionInWorld);
      this.baseOrientationInWorld.set(other.baseOrientationInWorld);
      this.jointPositions.set(other.jointPositions);
      this.jointVelocities.set(other.jointVelocities);
      this.baseLinearRateInBaseFrame.set(other.baseLinearRateInBaseFrame);
      this.baseAngularRateInBaseFrame.set(other.baseAngularRateInBaseFrame);
   }

   public Pose3DReadOnly getBasePoseInWorld()
   {
      return basePoseInWorld;
   }

   public Point3DReadOnly getBasePositionInWorld()
   {
      return basePositionInWorld;
   }

   public QuaternionReadOnly getBaseOrientationInWorld()
   {
      return baseOrientationInWorld;
   }

   public Vector3DReadOnly getBaseLinearRateInBaseFrame()
   {
      return baseLinearRateInBaseFrame;
   }

   public Vector3DReadOnly getBaseAngularRateInBaseFrame()
   {
      return baseAngularRateInBaseFrame;
   }

   public DMatrixRMaj getJointPositions()
   {
      return jointPositions;
   }

   public DMatrixRMaj getJointVelocities()
   {
      return jointVelocities;
   }
}
