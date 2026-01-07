package us.ihmc.simulationConstructionSetTools.util.visualizers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;

public class JointAxisVisualizer implements RobotController, SCS2YoGraphicHolder
{
   private final  String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final List<ReferenceFrame> referenceFrames = new ArrayList<>();
   private final List<YoFramePose3D> framePoses = new ArrayList<>();
   private final List<YoGraphicCoordinateSystem3DDefinition> graphics = new ArrayList<>();

   public JointAxisVisualizer(RigidBodyBasics rootBody, double length)
   {
      List<JointBasics> jointStack = new ArrayList<JointBasics>(rootBody.getChildrenJoints());
      while (!jointStack.isEmpty())
      {
         JointBasics joint = jointStack.get(0);
         if(joint instanceof OneDoFJointBasics)
         {
            FrameVector3DReadOnly jAxis=((OneDoFJointBasics)joint).getJointAxis();
            ReferenceFrame referenceFrame = GeometryTools.constructReferenceFrameFromPointAndZAxis(joint.getName()+"JointAxis", new FramePoint3D(jAxis.getReferenceFrame()), new FrameVector3D(jAxis.getReferenceFrame(),jAxis));
            YoFramePose3D framePose = new YoFramePose3D(referenceFrame.getName(), ReferenceFrame.getWorldFrame(), registry);
            graphics.add(newYoGraphicCoordinateSystem3D(referenceFrame.getName(), framePose, length, ColorDefinitions.Gold()));
            referenceFrames.add(referenceFrame);
            framePoses.add(framePose);
         }
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
   public String getDescription()
   {
      return getName();
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
