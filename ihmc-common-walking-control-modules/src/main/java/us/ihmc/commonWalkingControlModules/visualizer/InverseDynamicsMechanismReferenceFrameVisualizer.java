package us.ihmc.commonWalkingControlModules.visualizer;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;

public class InverseDynamicsMechanismReferenceFrameVisualizer implements Controller, SCS2YoGraphicHolder
{
   public enum FrameToDisplay
   {
      BODY_FIXED_FRAME, AFTER_JOINT_FRAME
   }

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final List<ReferenceFrame> referenceFrames = new ArrayList<>();
   private final List<YoFramePose3D> framePoses = new ArrayList<>();
   private final List<YoGraphicCoordinateSystem3DDefinition> graphics = new ArrayList<>();

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, double length, FrameToDisplay frameToDisplay, YoRegistry parentRegistry)
   {
      this(rootBody, length, frameToDisplay);
      parentRegistry.addChild(registry);
   }

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, double length)
   {
      this(rootBody, length, FrameToDisplay.BODY_FIXED_FRAME);
   }

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, double length, FrameToDisplay frameToDisplay)
   {
      List<JointBasics> jointStack = new ArrayList<JointBasics>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         JointBasics joint = jointStack.get(0);
         ReferenceFrame referenceFrame;
         switch(frameToDisplay)
         {
            case BODY_FIXED_FRAME:
               referenceFrame = joint.getSuccessor().getBodyFixedFrame();
               break;
            case AFTER_JOINT_FRAME:
               referenceFrame= joint.getFrameAfterJoint();
               break;
            default:
                  throw new RuntimeException("Invalid frame to display: " + frameToDisplay);
            
         }
         YoFramePose3D framePose = new YoFramePose3D(referenceFrame.getName(), ReferenceFrame.getWorldFrame(), registry);
         graphics.add(newYoGraphicCoordinateSystem3D(referenceFrame.getName(), framePose, length, ColorDefinitions.Gold()));
         referenceFrames.add(referenceFrame);
         framePoses.add(framePose);

         List<? extends JointBasics> childrenJoints = joint.getSuccessor().getChildrenJoints();
         jointStack.addAll(childrenJoints);
         jointStack.remove(joint);
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
      return name;
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < framePoses.size(); i++)
         framePoses.get(i).setFromReferenceFrame(referenceFrames.get(i));
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      graphics.forEach(group::addChild);
      return group;
   }
}
