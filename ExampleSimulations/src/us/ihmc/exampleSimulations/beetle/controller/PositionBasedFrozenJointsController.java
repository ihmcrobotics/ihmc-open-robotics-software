package us.ihmc.exampleSimulations.beetle.controller;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PositionBasedFrozenJointsController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final FullRobotModel fullRobotModel;
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable kp = new DoubleYoVariable("kp", registry);
   private final DoubleYoVariable kd = new DoubleYoVariable("kd", registry);

   public PositionBasedFrozenJointsController(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      kp.set(1000.0);
      kd.set(50.0);
   }


   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      for( OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         double desired = 0.0;
         double tau = kp.getDoubleValue() * (desired - joint.getQ()) + kd.getDoubleValue() * (desired - joint.getQd());
         joint.setTau(tau);
      }
   }
}
