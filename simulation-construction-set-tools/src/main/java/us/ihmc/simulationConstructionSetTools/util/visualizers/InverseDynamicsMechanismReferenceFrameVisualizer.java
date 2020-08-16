package us.ihmc.simulationConstructionSetTools.util.visualizers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.util.RobotController;

public class InverseDynamicsMechanismReferenceFrameVisualizer implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final List<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry,
         double length)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
      List<JointBasics> jointStack = new ArrayList<JointBasics>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         JointBasics joint = jointStack.get(0);
         ReferenceFrame referenceFrame = joint.getSuccessor().getBodyFixedFrame();
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
      for (YoGraphicReferenceFrame yoGraphicReferenceFrame : yoGraphicReferenceFrames)
      {
         yoGraphicReferenceFrame.update();
      }
   }
}
