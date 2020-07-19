package us.ihmc.wholeBodyController.concurrent;


import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.RewoundListener;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FullRobotModelRootJointRewinder implements RewoundListener
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FullRobotModel fullRobotModel;
   
   private final YoFrameVector3D yoRootJointTranslation = new YoFrameVector3D("yoRootJointTranslation", ReferenceFrame.getWorldFrame(), registry);
   private final Vector3D rootJointTranslation = new Vector3D();
   private final YoFrameQuaternion yoRootJointRotation = new YoFrameQuaternion("rootJointRotation", ReferenceFrame.getWorldFrame(), registry);
   private final Quaternion rootJointRotation = new Quaternion();

   public FullRobotModelRootJointRewinder(FullRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      parentRegistry.addChild(registry);
   }
   
   public void recordCurrentState()
   {
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      
      rootJointTranslation.set(rootJoint.getJointPose().getPosition());
      rootJointRotation.set(rootJoint.getJointPose().getOrientation());

      yoRootJointTranslation.set(rootJointTranslation);
      yoRootJointRotation.set(rootJointRotation);
   }

   @Override
   public void notifyOfRewind()
   {      
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      
      rootJoint.setJointPosition(yoRootJointTranslation);
      rootJoint.setJointOrientation(yoRootJointRotation);
   }
}

