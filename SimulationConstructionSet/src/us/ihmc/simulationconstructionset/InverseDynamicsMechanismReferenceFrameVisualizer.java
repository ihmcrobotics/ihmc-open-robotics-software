package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class InverseDynamicsMechanismReferenceFrameVisualizer implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final List<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   public InverseDynamicsMechanismReferenceFrameVisualizer(RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry,
         double length)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
      List<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint joint = jointStack.get(0);
         ReferenceFrame referenceFrame = joint.getSuccessor().getBodyFixedFrame();
         YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(referenceFrame, registry, length);
         yoGraphicsList.add(yoGraphicReferenceFrame);
         yoGraphicReferenceFrames.add(yoGraphicReferenceFrame);
         List<InverseDynamicsJoint> childrenJoints = joint.getSuccessor().getChildrenJoints();
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
   public YoVariableRegistry getYoVariableRegistry()
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
