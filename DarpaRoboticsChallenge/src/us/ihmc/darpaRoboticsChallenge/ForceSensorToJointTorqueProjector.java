package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

import us.ihmc.simulationconstructionset.robotController.RobotController;

/*
 * Compute YoVariables of ForceSensor measurement projected onto it's predecessor joints. Useful for checking sensor consistence.
 */
public class ForceSensorToJointTorqueProjector implements  RobotController
{   
   private final ForceSensorData forceSensorData;
   private final YoVariableRegistry registry;
   private final FrameVector tempFrameVector= new FrameVector();


   private final Wrench tempWrench = new Wrench();
   private final ArrayList<Pair<FrameVector,DoubleYoVariable>> yoTorqueInJoints;
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
         yoTorqueInJoints.add(new Pair<>(jAxis,new DoubleYoVariable("NegGRFWrenchIn"+ currentBody.getParentJoint().getName(), registry)));
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
         Pair<FrameVector, DoubleYoVariable> pair = yoTorqueInJoints.get(i);
         FrameVector jointAxis = pair.first();
         DoubleYoVariable torqueAboutJointAxis = pair.second();

         tempWrench.changeFrame(jointAxis.getReferenceFrame());
         tempFrameVector.setToZero(tempWrench.getExpressedInFrame());
         tempWrench.packAngularPart(tempFrameVector);
         torqueAboutJointAxis.set(-tempFrameVector.dot(jointAxis));
      }

   }

}
