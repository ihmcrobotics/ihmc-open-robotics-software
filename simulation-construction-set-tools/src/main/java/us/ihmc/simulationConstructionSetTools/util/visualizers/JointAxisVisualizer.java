package us.ihmc.simulationConstructionSetTools.util.visualizers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointAxisVisualizer implements RobotController
{
   private final  String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final List<YoGraphicReferenceFrame> yoGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();
   
   public JointAxisVisualizer(RigidBodyBasics rootBody, YoGraphicsListRegistry yoGraphicsListRegistry, double length)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
      List<JointBasics> jointStack = new ArrayList<JointBasics>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         JointBasics joint = jointStack.get(0);
         if(joint instanceof OneDoFJointBasics)
         {
            FrameVector3DReadOnly jAxis=((OneDoFJointBasics)joint).getJointAxis();
            ReferenceFrame referenceFrame = GeometryTools.constructReferenceFrameFromPointAndZAxis(joint.getName()+"JointAxis", new FramePoint3D(jAxis.getReferenceFrame()), new FrameVector3D(jAxis.getReferenceFrame(),jAxis));
            YoGraphicReferenceFrame yoGraphicReferenceFrame = new YoGraphicReferenceFrame(referenceFrame , registry, false, length, YoAppearance.Gold());
            yoGraphicsList.add(yoGraphicReferenceFrame);
            yoGraphicReferenceFrames.add(yoGraphicReferenceFrame);
         }
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
