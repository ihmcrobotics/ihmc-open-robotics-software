package us.ihmc.commonWalkingControlModules.packetConsumers;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class UserDesiredHandstepProvider implements HandstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userHandstepTakeIt = new BooleanYoVariable("userHandstepTakeIt", registry);
   private final YoFramePoint userHandstepPosition = new YoFramePoint("userHandstepPosition", ReferenceFrame.getWorldFrame(), registry);
   private final EnumYoVariable<RobotSide> userHandstepRobotSide = new EnumYoVariable<RobotSide>("userHandstepRobotSide", registry, RobotSide.class);
   private final YoFrameVector userHandstepNormal = new YoFrameVector("userHandstepNormal", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable userHandstepRotationAboutNormal = new DoubleYoVariable("userHandstepRotationAboutNormal", registry);

   private final DynamicGraphicCoordinateSystem userDesiredHandstepCoordinateSystem;

   private final FullRobotModel fullRobotModel;

   public UserDesiredHandstepProvider(FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry,
                                      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      userHandstepTakeIt.set(false);
      userHandstepNormal.set(-1.0, 0.0, 0.0);
      userHandstepRobotSide.set(RobotSide.LEFT);

      userDesiredHandstepCoordinateSystem = new DynamicGraphicCoordinateSystem("userHandstepViz", "", parentRegistry, 0.3);

      VariableChangedListener listener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            userDesiredHandstepCoordinateSystem.setTransformToWorld(computeHandstepTransform(false));
         }
      };

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("UserDesiredHandstep", userDesiredHandstepCoordinateSystem);

      userHandstepNormal.attachVariableChangedListener(listener);
      userHandstepRotationAboutNormal.addVariableChangedListener(listener);
      userHandstepPosition.attachVariableChangedListener(listener);

      parentRegistry.addChild(registry);

      this.fullRobotModel = fullRobotModel;
   }

   public Handstep getDesiredHandstep(RobotSide robotSide)
   {
      if (!userHandstepTakeIt.getBooleanValue())
         return null;
      if (userHandstepRobotSide.getEnumValue() != robotSide)
         return null;

      Transform3D transformOne = computeHandstepTransform(true);

      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transformOne);

      RigidBody hand = fullRobotModel.getHand(robotSide);
      Handstep handstep = new Handstep(hand, framePose);
      userHandstepTakeIt.set(false);

      return handstep;
   }

   private Transform3D computeHandstepTransform(boolean rotateZIntoX)
   {
      Vector3d normal = userHandstepNormal.getVector3dCopy();
      normal.normalize();
      AxisAngle4d rotationAxisAngle = new AxisAngle4d();
      GeometryTools.getRotationBasedOnNormal(rotationAxisAngle, normal);

      AxisAngle4d rotationAboutNormal = new AxisAngle4d(normal, userHandstepRotationAboutNormal.getDoubleValue());

      Transform3D transformOne = new Transform3D();
      transformOne.set(rotationAboutNormal);

      Transform3D transformTwo = new Transform3D();
      transformTwo.set(rotationAxisAngle);
      transformOne.mul(transformTwo);

      if (rotateZIntoX)
      {
         Transform3D transformThree = new Transform3D();
         transformThree.rotY(Math.PI/2.0);
         transformOne.mul(transformThree);
      }
      
      transformOne.setTranslation(userHandstepPosition.getVector3dCopy());

      return transformOne;
   }

   public boolean checkForNewHandstep(RobotSide robotSide)
   {
      if (!userHandstepTakeIt.getBooleanValue())
         return false;
      if (userHandstepRobotSide.getEnumValue() != robotSide)
         return false;
      return true;
   }

}
