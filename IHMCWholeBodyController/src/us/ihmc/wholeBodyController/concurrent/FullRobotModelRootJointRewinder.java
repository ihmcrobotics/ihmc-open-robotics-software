package us.ihmc.wholeBodyController.concurrent;


import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.model.BaseFullRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.RewoundListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class FullRobotModelRootJointRewinder implements RewoundListener
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final BaseFullRobotModel fullRobotModel;
   
   private final YoFrameVector yoRootJointTranslation = new YoFrameVector("yoRootJointTranslation", ReferenceFrame.getWorldFrame(), registry);
   private final Vector3d rootJointTranslation = new Vector3d();
   private final YoFrameQuaternion yoRootJointRotation = new YoFrameQuaternion("rootJointRotation", ReferenceFrame.getWorldFrame(), registry);
   private final Quat4d rootJointRotation = new Quat4d();

   public FullRobotModelRootJointRewinder(BaseFullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      parentRegistry.addChild(registry);
   }
   
   public void recordCurrentState()
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      SixDoFJointReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      
      rootJointFrame.getTraslation(rootJointTranslation);
      rootJointFrame.getRotation(rootJointRotation);

      yoRootJointTranslation.set(rootJointTranslation);
      yoRootJointRotation.set(rootJointRotation);
   }

   @Override
   public void wasRewound()
   {      
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      
      yoRootJointTranslation.get(rootJointTranslation);
      rootJoint.setPosition(rootJointTranslation);
      
      yoRootJointRotation.get(rootJointRotation);
      rootJoint.setRotation(rootJointRotation);
   }
}

