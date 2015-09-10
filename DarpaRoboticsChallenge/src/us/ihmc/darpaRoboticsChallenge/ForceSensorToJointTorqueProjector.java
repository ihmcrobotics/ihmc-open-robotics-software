package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

/*
 * Compute YoVariables of ForceSensor measurement projected onto it's predecessor joints. Useful for checking sensor consistence.
 */
public class ForceSensorToJointTorqueProjector implements  RobotController
{   
   private final ForceSensorData forceSensorData;
   private final YoVariableRegistry registry;
   private final FrameVector tempFrameVector= new FrameVector();


   private final Wrench tempWrench = new Wrench();
   private final ArrayList<ImmutablePair<FrameVector,DoubleYoVariable>> yoTorqueInJoints;
   private final int numberOfJointFromSensor = 2;

   public ForceSensorToJointTorqueProjector(String namePrefix, ForceSensorData forceSensorData, RigidBody sensorLinkBody) 
   {
      registry = new YoVariableRegistry(namePrefix+getClass().getSimpleName());

      this.forceSensorData = forceSensorData;

      //ground reaction wrench on joints
      yoTorqueInJoints = new ArrayList<>();
      RigidBody currentBody = sensorLinkBody;
      for(int i=0;i<numberOfJointFromSensor;i++)
      {
         FrameVector jAxis = ((OneDoFJoint)currentBody.getParentJoint()).getJointAxis();
         yoTorqueInJoints.add(new ImmutablePair<>(jAxis,new DoubleYoVariable("NegGRFWrenchIn"+ currentBody.getParentJoint().getName(), registry)));
         currentBody=currentBody.getParentJoint().getPredecessor();
      }
   }
   @Override
   public void initialize()
   {
      doControl();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
      forceSensorData.packWrench(tempWrench);
      for(int i = 0; i < yoTorqueInJoints.size(); i++)
      {
         ImmutablePair<FrameVector, DoubleYoVariable> pair = yoTorqueInJoints.get(i);
         FrameVector jointAxis = pair.getLeft();
         DoubleYoVariable torqueAboutJointAxis = pair.getRight();

         tempWrench.changeFrame(jointAxis.getReferenceFrame());
         tempFrameVector.setToZero(tempWrench.getExpressedInFrame());
         tempWrench.packAngularPart(tempFrameVector);
         torqueAboutJointAxis.set(-tempFrameVector.dot(jointAxis));
      }

   }

}
