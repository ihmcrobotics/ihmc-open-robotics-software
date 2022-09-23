package us.ihmc.footstepPlanning.baselinePlanner;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;

import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.nodes.Tag;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;

public class BaselineFootstepPlannerParameters
{

   protected double stanceWidth = 0.24; // m
   protected double stanceSplay = 0.00; // rad
   protected double lateralVelocityStanceScaling = 0.3; // m / (m / s)
   protected double turningVelocityStanceScaling = 0.4; // rad / (rad / s)
   protected double swingDuration = 0.6; // s
   protected double minimumTransferDuration = 0.2; // s
   protected double minimumForwardStride = 0.025; // m
   protected double minimumLateralStride = 0.4; // m
   protected double minimumTurningStride = 0.2; // rad
   protected double forwardSymmetryWeight = 1.0;
   protected double lateralSymmetryWeight = 1.0;
   protected double turningSymmetryWeight = 1.0;
   protected double stancePositionAlignmentThreshold = 0.03; // m
   protected double stanceRotationAlignmentThreshold = 0.10; // rad

   public BaselineFootstepPlannerParameters()
   {

   }

   public BaselineFootstepPlannerParameters(FootstepPlannerParametersBasics footstepPlannerParametersBasics)
   {
      this();
      this.minimumForwardStride = footstepPlannerParametersBasics.getIdealFootstepLength();
//      this.minimumLateralStride = footstepPlannerParametersBasics.getIdealSideStepWidth();
   }

   public BaselineFootstepPlannerParameters(BaselineFootstepPlannerParameters other)
   {
      set(other);
   }

   public final void set(BaselineFootstepPlannerParameters other)
   {
      this.stanceWidth = other.stanceWidth;
      this.stanceSplay = other.stanceSplay;
      this.lateralVelocityStanceScaling = other.lateralVelocityStanceScaling;
      this.turningVelocityStanceScaling = other.turningVelocityStanceScaling;
      this.swingDuration = other.swingDuration;
      this.minimumTransferDuration = other.minimumTransferDuration;
      this.minimumForwardStride = other.minimumForwardStride;
      this.minimumLateralStride = other.minimumLateralStride;
      this.minimumTurningStride = other.minimumTurningStride;
      this.forwardSymmetryWeight = other.forwardSymmetryWeight;
      this.lateralSymmetryWeight = other.lateralSymmetryWeight;
      this.turningSymmetryWeight = other.turningSymmetryWeight;
      this.stancePositionAlignmentThreshold = other.stancePositionAlignmentThreshold;
      this.stanceRotationAlignmentThreshold = other.stanceRotationAlignmentThreshold;
   }

   public final boolean loadFromYaml(Reader reader)
   {
      Yaml yaml = new Yaml(new Constructor(BaselineFootstepPlannerParameters.class));
      BaselineFootstepPlannerParameters parameters = (BaselineFootstepPlannerParameters) yaml.load(reader);
      if (parameters == null)
         return false;
      this.set(parameters);
      return true;
   }

   public final String dumpToYaml()
   {
      DumperOptions options = new DumperOptions();
      options.setDefaultFlowStyle(DumperOptions.FlowStyle.BLOCK);
      Yaml yaml = new Yaml(options);
      return yaml.dumpAs(this, Tag.MAP, null);
   }

   public final double getStanceWidth()
   {
      return stanceWidth;
   }

   public final void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public final double getStanceSplay()
   {
      return stanceSplay;
   }

   public final void setStanceSplay(double stanceSplay)
   {
      this.stanceSplay = stanceSplay;
   }

   public final double getLateralVelocityStanceScaling()
   {
      return lateralVelocityStanceScaling;
   }

   public final void setLateralVelocityStanceScaling(double lateralVelocityStanceScaling)
   {
      this.lateralVelocityStanceScaling = lateralVelocityStanceScaling;
   }

   public final double getTurningVelocityStanceScaling()
   {
      return turningVelocityStanceScaling;
   }

   public final void setTurningVelocityStanceScaling(double turningVelocityStanceScaling)
   {
      this.turningVelocityStanceScaling = turningVelocityStanceScaling;
   }

   public final double getSwingDuration()
   {
      return swingDuration;
   }

   public final void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public final double getMinimumTransferDuration()
   {
      return minimumTransferDuration;
   }

   public final void setMinimumTransferDuration(double minimumTransferDuration)
   {
      this.minimumTransferDuration = minimumTransferDuration;
   }

   public final double getMinimumForwardStride()
   {
      return minimumForwardStride;
   }

   public final void setMinimumForwardStride(double minimumForwardStride)
   {
      this.minimumForwardStride = minimumForwardStride;
   }

   public final double getMinimumLateralStride()
   {
      return minimumLateralStride;
   }

   public final void setMinimumLateralStride(double minimumLateralStride)
   {
      this.minimumLateralStride = minimumLateralStride;
   }

   public final double getMinimumTurningStride()
   {
      return minimumTurningStride;
   }

   public final void setMinimumTurningStride(double minimumTurningStride)
   {
      this.minimumTurningStride = minimumTurningStride;
   }

   public final double getForwardSymmetryWeight()
   {
      return forwardSymmetryWeight;
   }

   public final void setForwardSymmetryWeight(double forwardSymmetryWeight)
   {
      this.forwardSymmetryWeight = forwardSymmetryWeight;
   }

   public final double getLateralSymmetryWeight()
   {
      return lateralSymmetryWeight;
   }

   public final void setLateralSymmetryWeight(double lateralSymmetryWeight)
   {
      this.lateralSymmetryWeight = lateralSymmetryWeight;
   }

   public final double getTurningSymmetryWeight()
   {
      return turningSymmetryWeight;
   }

   public final void setTurningSymmetryWeight(double turningSymmetryWeight)
   {
      this.turningSymmetryWeight = turningSymmetryWeight;
   }

   public final double getStancePositionAlignmentThreshold()
   {
      return stancePositionAlignmentThreshold;
   }

   public final void setStancePositionAlignmentThreshold(double stancePositionAlignmentThreshold)
   {
      this.stancePositionAlignmentThreshold = stancePositionAlignmentThreshold;
   }

   public final double getStanceRotationAlignmentThreshold()
   {
      return stanceRotationAlignmentThreshold;
   }

   public final void setStanceRotationAlignmentThreshold(double stanceRotationAlignmentThreshold)
   {
      this.stanceRotationAlignmentThreshold = stanceRotationAlignmentThreshold;
   }
}
