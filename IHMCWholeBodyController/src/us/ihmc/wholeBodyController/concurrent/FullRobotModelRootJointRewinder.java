package us.ihmc.wholeBodyController.concurrent;


import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJointReferenceFrame;

public class FullRobotModelRootJointRewinder implements RewoundListener
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final FullRobotModel fullRobotModel;
   
   private final YoFrameVector yoRootJointTranslation = new YoFrameVector("yoRootJointTranslation", ReferenceFrame.getWorldFrame(), registry);
   private final Vector3d rootJointTranslation = new Vector3d();
   private final YoFrameQuaternion yoRootJointRotation = new YoFrameQuaternion("rootJointRotation", ReferenceFrame.getWorldFrame(), registry);
   private final Quat4d rootJointRotation = new Quat4d();

   public FullRobotModelRootJointRewinder(FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      parentRegistry.addChild(registry);
   }
   
   public void recordCurrentState()
   {
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      FloatingInverseDynamicsJointReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      
      rootJointFrame.getTraslation(rootJointTranslation);
      rootJointFrame.getRotation(rootJointRotation);

      yoRootJointTranslation.set(rootJointTranslation);
      yoRootJointRotation.set(rootJointRotation);
   }

   @Override
   public void wasRewound()
   {      
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      
      yoRootJointTranslation.get(rootJointTranslation);
      rootJoint.setPosition(rootJointTranslation);
      
      yoRootJointRotation.get(rootJointRotation);
      rootJoint.setRotation(rootJointRotation);
   }
}

