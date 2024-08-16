package us.ihmc.rdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

public class RDXIKSolverColors
{
   public static final String GOOD_QUALITY_COLOR_CODE = "0x4B61D1";
   public static final String BAD_QUALITY_COLOR_CODE = "0xD14B4B";

   public static final ColorDefinition GOOD_QUALITY_COLOR_DEFINITION = ColorDefinitions.parse(GOOD_QUALITY_COLOR_CODE).derive(0.0, 1.0, 1.0, 0.5);
   public static final ColorDefinition BAD_QUALITY_COLOR_DEFINITION = ColorDefinitions.parse(BAD_QUALITY_COLOR_CODE).derive(0.0, 1.0, 1.0, 0.5);

   public static final Color GOOD_QUALITY_COLOR = RDXVisualTools.toColor(GOOD_QUALITY_COLOR_DEFINITION);
   public static final Color BAD_QUALITY_COLOR = RDXVisualTools.toColor(BAD_QUALITY_COLOR_DEFINITION);

   public static Color getColor(double solutionQuality)
   {
      return solutionQuality > ArmIKSolver.GOOD_QUALITY_MAX ? RDXIKSolverColors.BAD_QUALITY_COLOR : RDXIKSolverColors.GOOD_QUALITY_COLOR;
   }

   public static Color getColor(boolean isSolutionGood)
   {
      return isSolutionGood ? RDXIKSolverColors.GOOD_QUALITY_COLOR : RDXIKSolverColors.BAD_QUALITY_COLOR;
   }
}
