package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/*
 * Compute YoVariables of ForceSensor measurement projected onto it's predecessor joints. Useful for checking sensor consistence.
 */
public class ForceSensorToJointTorqueProjector implements RobotController
{
   private final ForceSensorData forceSensorData;
   private final YoRegistry registry;
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   private final Wrench tempWrench = new Wrench();
   private final ArrayList<ImmutablePair<FrameVector3D, YoDouble>> yoTorqueInJoints;
   private final int numberOfJointFromSensor = 2;

   public ForceSensorToJointTorqueProjector(String namePrefix, ForceSensorData forceSensorData, RigidBodyBasics sensorLinkBody)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      this.forceSensorData = forceSensorData;

      //ground reaction wrench on joints
      yoTorqueInJoints = new ArrayList<>();
      RigidBodyBasics currentBody = sensorLinkBody;
      for (int i = 0; i < numberOfJointFromSensor; i++)
      {
         FrameVector3D jAxis = new FrameVector3D(((OneDoFJointBasics) currentBody.getParentJoint()).getJointAxis());
         yoTorqueInJoints.add(new ImmutablePair<>(jAxis, new YoDouble("NegGRFWrenchIn" + currentBody.getParentJoint().getName(), registry)));
         currentBody = currentBody.getParentJoint().getPredecessor();
      }
   }

   @Override
   public void initialize()
   {
      doControl();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getClass().getName();
   }

   @Override
   public void doControl()
   {
      tempWrench.setIncludingFrame(forceSensorData.getWrench());
      for (int i = 0; i < yoTorqueInJoints.size(); i++)
      {
         ImmutablePair<FrameVector3D, YoDouble> pair = yoTorqueInJoints.get(i);
         FrameVector3D jointAxis = pair.getLeft();
         YoDouble torqueAboutJointAxis = pair.getRight();

         tempWrench.changeFrame(jointAxis.getReferenceFrame());
         tempFrameVector.setToZero(tempWrench.getReferenceFrame());
         tempFrameVector.set(tempWrench.getAngularPart());
         torqueAboutJointAxis.set(-tempFrameVector.dot(jointAxis));
      }

   }

}
