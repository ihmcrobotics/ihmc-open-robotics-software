package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;

public class AntiGravityJointTorquesVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> antiGravityJointTorques;
   
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SideDependentList<WrenchBasedFootSwitch> wrenchBasedFootSwitches;
   private final InverseDynamicsJoint[] allJoints;
   private final OneDoFJoint[] allOneDoFJoints;
   private final Wrench tempWrench = new Wrench();

   public AntiGravityJointTorquesVisualizer(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, SideDependentList<WrenchBasedFootSwitch> wrenchBasedFootSwitches, YoVariableRegistry parentRegistry, double gravity)
   {
      SpatialAccelerationVector rootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), gravity);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(ReferenceFrame.getWorldFrame(), rootAcceleration, new LinkedHashMap<RigidBody, Wrench>(), new ArrayList<InverseDynamicsJoint>(), false, false, twistCalculator);
      this.wrenchBasedFootSwitches = wrenchBasedFootSwitches;
      allJoints = ScrewTools.computeSubtreeJoints(inverseDynamicsCalculator.getSpatialAccelerationCalculator().getRootBody());
      allOneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);
      
      antiGravityJointTorques = new LinkedHashMap<>(allOneDoFJoints.length);
      
      for (int i = 0; i < allOneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = allOneDoFJoints[i];
         DoubleYoVariable antiGravityJointTorque = new DoubleYoVariable("antiGravity_tau_" + oneDoFJoint.getName(), registry);
         antiGravityJointTorques.put(oneDoFJoint, antiGravityJointTorque);
      }
      parentRegistry.addChild(registry);
   }

   public void computeAntiGravityJointTorques()
   {
      reset();
      
      setFootMeasuredWrenches();
      inverseDynamicsCalculator.compute();
      for (int i = 0; i < allOneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = allOneDoFJoints[i];
         antiGravityJointTorques.get(oneDoFJoint).set(oneDoFJoint.getTau());
         oneDoFJoint.setTau(0.0);
      }
      reset();
   }

   private void setFootMeasuredWrenches()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         WrenchBasedFootSwitch wrenchBasedFootSwitch = wrenchBasedFootSwitches.get(robotSide);
         wrenchBasedFootSwitch.computeAndPackFootWrench(tempWrench);
         RigidBody foot = wrenchBasedFootSwitch.getContactablePlaneBody().getRigidBody();
         tempWrench.changeBodyFrameAttachedToSameBody(foot.getBodyFixedFrame());
         tempWrench.changeFrame(foot.getBodyFixedFrame());
         inverseDynamicsCalculator.setExternalWrench(foot, tempWrench);
      }
   }

   private void reset()
   {
      inverseDynamicsCalculator.reset();
   }
}
