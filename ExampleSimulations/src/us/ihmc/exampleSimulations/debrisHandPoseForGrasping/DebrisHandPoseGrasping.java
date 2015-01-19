package us.ihmc.exampleSimulations.debrisHandPoseForGrasping;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class DebrisHandPoseGrasping
{

   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public DebrisHandPoseGrasping()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
      final YoFramePose debrisPose = new YoFramePose("debris", worldFrame, registry);
      final YoFramePose initialHandPose = new YoFramePose("initialHand", worldFrame, registry);
      final YoFramePose graspingPose = new YoFramePose("grasping", worldFrame, registry);
      
      debrisPose.setXYZ(0.2, 0.0, 0.5);
      debrisPose.setYawPitchRoll(0.0, 0.0, 0.0);
      
      initialHandPose.setXYZ(0.0, 0.0, 0.5);
      initialHandPose.setYawPitchRoll(0.0, 0.0, 0.0);
      
      final ReferenceFrame handFrame = new ReferenceFrame("handFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            initialHandPose.getFramePoseIncludingFrame(localFramePose);
            localFramePose.getPose(transformToParent);
         }
      };

      final ReferenceFrame debrisFrame = new ReferenceFrame("debrisFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            debrisPose.getFramePoseIncludingFrame(localFramePose);
            localFramePose.getPose(transformToParent);
         }
      };

      final ReferenceFrame desiredGraspFrame = new ReferenceFrame("desiredGraspFrame", worldFrame)
      {
         private static final long serialVersionUID = 1L;
         private FramePose localFramePose = new FramePose();
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            graspingPose.getFramePoseIncludingFrame(localFramePose);
            localFramePose.getPose(transformToParent);
         }
      };
      
      final YoGraphicReferenceFrame handFrameViz = new YoGraphicReferenceFrame(handFrame, registry, 0.3, YoAppearance.Black());
      final YoGraphicReferenceFrame debrisFrameViz = new YoGraphicReferenceFrame(debrisFrame, registry, 0.3, YoAppearance.Blue());
      final YoGraphicReferenceFrame desiredGraspFrameViz = new YoGraphicReferenceFrame(desiredGraspFrame, registry, 0.4, YoAppearance.Yellow());
      
      graphicsListRegistry.registerYoGraphic("Frames", handFrameViz);
      graphicsListRegistry.registerYoGraphic("Frames", debrisFrameViz);
      graphicsListRegistry.registerYoGraphic("Frames", desiredGraspFrameViz);

      setupListener(debrisPose, debrisFrame, debrisFrameViz);
      setupListener(initialHandPose, handFrame, handFrameViz);
      setupListener(graspingPose, desiredGraspFrame, desiredGraspFrameViz);
      
      final BooleanYoVariable computeDesiredGraspPose = new BooleanYoVariable("computeDesiredGraspPose", registry);
      
      computeDesiredGraspPose.addVariableChangedListener(new VariableChangedListener()
      {
         private FramePose localFramePose = new FramePose();

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (!computeDesiredGraspPose.getBooleanValue())
               return;
            
            computeDesiredGraspPose.set(false, false);
            
            RigidBodyTransform debrisTransform = new RigidBodyTransform();
            debrisPose.getFramePose(localFramePose);
            localFramePose.getPose(debrisTransform);
            Quat4d desiredGraspOrientationToPack = new Quat4d();
            computeDesiredGraspOrientation(debrisTransform, handFrame, desiredGraspOrientationToPack);
            Point3d position = new Point3d();
            debrisPose.getPosition().get(position);
            graspingPose.setPosition(position);
            graspingPose.setOrientation(desiredGraspOrientationToPack);
         }
      });
      
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      
      setupSliderBoard(scs, debrisPose);
      
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }


   private void setupSliderBoard(SimulationConstructionSet scs, YoFramePose debrisPose)
   {
      final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);
      
      int i = 1;
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoX(), -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoY(), -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getPosition().getYoZ(), -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getYaw(), -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getPitch(), -0.5, 0.5);
      sliderBoardConfigurationManager.setSlider(i++, debrisPose.getOrientation().getRoll(), -0.5, 0.5);
   }


   private void setupListener(final YoFramePose pose, final ReferenceFrame frameToUpdate, final YoGraphic yoGraphicToUpdate)
   {
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            frameToUpdate.update();
            yoGraphicToUpdate.update();
         }
      };
      pose.attachVariableChangedListener(variableChangedListener);
      variableChangedListener.variableChanged(null);
   }
   

   private void computeDesiredGraspOrientation(RigidBodyTransform debrisTransform, ReferenceFrame handFrame, Quat4d desiredGraspOrientationToPack)
   {
      FramePose debrisPoseInHandControlFrame = new FramePose(ReferenceFrame.getWorldFrame(), debrisTransform);
      debrisPoseInHandControlFrame.changeFrame(handFrame);

      FramePose debrisPoseInHandControlFramePiRoll = new FramePose(ReferenceFrame.getWorldFrame(), debrisTransform);
      debrisPoseInHandControlFramePiRoll.changeFrame(handFrame);
      double piRollRoll = debrisPoseInHandControlFramePiRoll.getRoll();
      piRollRoll += Math.PI;
      AngleTools.trimAngleMinusPiToPi(piRollRoll);

      //matrix3d
      
      AxisAngle4d axisAngle = new AxisAngle4d();
      AxisAngle4d axisAnglePiRoll = new AxisAngle4d();
      debrisPoseInHandControlFrame.getOrientation(axisAngle);
      debrisPoseInHandControlFramePiRoll.getOrientation(axisAnglePiRoll);
      
      System.out.println(debrisPoseInHandControlFrame);
      System.out.println(debrisPoseInHandControlFramePiRoll);

      if (axisAngle.getAngle() > axisAnglePiRoll.getAngle())
         debrisPoseInHandControlFrame.getOrientation(desiredGraspOrientationToPack);
      else
         debrisPoseInHandControlFramePiRoll.getOrientation(desiredGraspOrientationToPack);
   }

   public static void main(String[] args)
   {
      new DebrisHandPoseGrasping();
   }
}
