package us.ihmc.gr00t;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Operator-facing diagnostics for the reusable humanoid GR00T executor. The YoVariables are
 * diagnostics, not the control source, and never expose mutable network buffers to render threads.
 */
public class Gr00tHumanoidDiagnostics
{
   private final String[] handTargetNames;
   protected final YoRegistry registry;

   /** Last valid policy wrist pose for each side, expressed in world coordinates. */
   protected final SideDependentList<YoPose3D> actionWristPoses;
   /** Last policy finger block for each side. */
   protected final SideDependentList<YoDouble[]> actionFingers;
   /** Total valid rows decoded from accepted bridge responses. */
   protected final YoLong numberOfActionsReceived;
   /** Total rows incorporated into validated IK plans. */
   protected final YoLong numberOfActionsTaken;

   protected Gr00tHumanoidDiagnostics(String registryName, String[] handTargetNames)
   {
      if (registryName == null || registryName.isBlank())
         throw new IllegalArgumentException("registryName must not be blank");
      this.handTargetNames = handTargetNames.clone();
      registry = new YoRegistry(registryName);
      actionWristPoses = new SideDependentList<>(side -> new YoPose3D(side.getLowerCaseName() + "ActionWristPose", registry));
      actionFingers = new SideDependentList<>(this::makeFingerVariables);
      numberOfActionsReceived = new YoLong("gr00tNumberOfActionsReceived", registry);
      numberOfActionsTaken = new YoLong("gr00tNumberOfActionsTaken", registry);
   }

   private YoDouble[] makeFingerVariables(RobotSide side)
   {
      YoDouble[] fingers = new YoDouble[handTargetNames.length];
      for (int i = 0; i < handTargetNames.length; i++)
         fingers[i] = new YoDouble(side.getLowerCaseName() + "ActionFinger" + handTargetNames[i], registry);
      return fingers;
   }

   public String[] getHandTargetNames()
   {
      return handTargetNames.clone();
   }

   public SideDependentList<YoPose3D> getActionWristPoses()
   {
      return actionWristPoses;
   }

   public SideDependentList<YoDouble[]> getActionFingers()
   {
      return actionFingers;
   }

   public long getNumberOfActionsReceived()
   {
      return numberOfActionsReceived.getValue();
   }

   public long getNumberOfActionsTaken()
   {
      return numberOfActionsTaken.getValue();
   }

}
