package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class InverseDynamicsMechanismReferenceFrameVisualizer implements RobotController
{
   public enum FrameToDisplay
   {
      BODY_FIXED_FRAME, AFTER_JOINT_FRAME
   }

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final List<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, double length, FrameToDisplay frameToDisplay,
                                                           YoRegistry parentRegistry)
   {
      this(rootBody, yoGraphicsListRegistry, length, frameToDisplay);
      parentRegistry.addChild(registry);
   }

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, double length)
   {
      this(rootBody, yoGraphicsListRegistry, length, FrameToDisplay.BODY_FIXED_FRAME);
   }

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry,
         double length, FrameToDisplay frameToDisplay)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
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
         YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(referenceFrame, registry, false, length);
         yoGraphicsList.add(yoGraphicReferenceFrame);
         yoGraphicReferenceFrames.add(yoGraphicReferenceFrame);
         List<? extends JointBasics> childrenJoints = joint.getSuccessor().getChildrenJoints();
         jointStack.addAll(childrenJoints);
         jointStack.remove(joint);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
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
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < yoGraphicReferenceFrames.size(); i++)
      {
         yoGraphicReferenceFrames.get(i).update();
      }
   }
}
