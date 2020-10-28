package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class JumpingCoPTrajectoryGeneratorState extends YoSaveableModuleState
{
   private final YoDouble finalTransferDuration;

   private final YoFramePoint2D initialCoP;

   private final JumpingGoalVariable jumpingGoal;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSole = new SideDependentList<>();
   private final SideDependentList<FixedFramePose3DBasics> footPoses = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> soleContactFrames = new SideDependentList<>();

   public JumpingCoPTrajectoryGeneratorState(YoRegistry registry)
   {
      jumpingGoal = new JumpingGoalVariable("", registry);
      registerStateToSave(jumpingGoal);

      finalTransferDuration = new YoDouble("finalTransferDuration", registry);
      registerVariableToSave(finalTransferDuration);

      initialCoP = new YoFramePoint2D("initialCoP", ReferenceFrame.getWorldFrame(), registry);
      YoSaveableModuleStateTools.registerYoTuple2DToSave(initialCoP, this);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePose3D footPose = new YoFramePose3D(robotSide.getCamelCaseName() + "FootPose", ReferenceFrame.getWorldFrame(), registry);
         YoSaveableModuleStateTools.registerYoFramePose3DToSave(footPose, this);
         footPoses.put(robotSide, footPose);

         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getCamelCaseName() + "SoleFrame", footPose);
         soleContactFrames.put(robotSide, soleFrame);

         footPose.attachVariableChangedListener(v -> soleFrame.setPoseAndUpdate(footPose));

         List<YoFramePoint2D> vertexBuffer = new ArrayList<>();
         String prefix = robotSide.getCamelCaseName() + "FootPolygonInSole";
         for (int i = 0; i < 6; i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "_" + i, soleFrame, registry);
            YoSaveableModuleStateTools.registerYoTuple2DToSave(vertex, this);
            vertexBuffer.add(vertex);
         }
         YoInteger numberOfVertices = new YoInteger(prefix + "NumVertices", registry);
         registerVariableToSave(numberOfVertices);
         YoFrameConvexPolygon2D footPolygonInSole = new YoFrameConvexPolygon2D(vertexBuffer,
                                                                               numberOfVertices,
                                                                               soleFrame);
         footPolygonInSole.clearAndUpdate();
         footPolygonsInSole.put(robotSide, footPolygonInSole);
      }
   }

   public void initializeStance(SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames,
                                SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      for (RobotSide robotSide : RobotSide.values)
         initializeStance(robotSide, feetInSoleZUpFrames.get(robotSide), soleFrames.get(robotSide));
   }

   public void initializeStance(RobotSide robotSide, FrameConvexPolygon2DReadOnly supportPolygon, ReferenceFrame soleFrame)
   {
      footPoses.get(robotSide).setFromReferenceFrame(soleFrame);
      footPolygonsInSole.get(robotSide).setMatchingFrame(supportPolygon, false);
   }

   public void setJumpingGoal(JumpingGoal jumpingGoal)
   {
      this.jumpingGoal.set(jumpingGoal);
   }

   public FramePoint2DReadOnly getInitialCoP()
   {
      return initialCoP;
   }

   public FramePose3DReadOnly getFootPose(RobotSide robotSide)
   {
      return footPoses.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInSole(RobotSide robotSide)
   {
      return footPolygonsInSole.get(robotSide);
   }

   public JumpingGoalVariable getJumpingGoal()
   {
      return jumpingGoal;
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration.getDoubleValue();
   }

   public void setFinalTransferDuration(double transferDuration)
   {
      finalTransferDuration.set(transferDuration);
   }

   public void clear()
   {
   }

   public void setInitialCoP(FramePoint3DReadOnly initialCoP)
   {
      this.initialCoP.set(initialCoP);
   }

   public void setInitialCoP(FramePoint2DReadOnly initialCoP)
   {
      this.initialCoP.set(initialCoP);
   }
}
