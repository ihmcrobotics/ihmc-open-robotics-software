package us.ihmc.gr00t;

import us.ihmc.robotics.robotSide.RobotSide;

/** Hand telemetry and optional policy/manual command output used by a humanoid GR00T task. */
public interface Gr00tHandController
{
   void update();

   boolean publishPolicyTargets(RobotSide side, double[] targetsRadians);

   boolean hasState(RobotSide side);

   double[] getJointPositions(RobotSide side);

   void requestGrip(RobotSide side, boolean close);

   void setEnabled(boolean enabled);

   boolean isEnabled();

   static Gr00tHandController noOp()
   {
      return new Gr00tHandController()
      {
         private boolean enabled;

         @Override public void update() { }
         @Override public boolean publishPolicyTargets(RobotSide side, double[] targetsRadians) { return false; }
         @Override public boolean hasState(RobotSide side) { return false; }
         @Override public double[] getJointPositions(RobotSide side) { return new double[0]; }
         @Override public void requestGrip(RobotSide side, boolean close) { }
         @Override public void setEnabled(boolean enabled) { this.enabled = enabled; }
         @Override public boolean isEnabled() { return enabled; }
      };
   }
}
