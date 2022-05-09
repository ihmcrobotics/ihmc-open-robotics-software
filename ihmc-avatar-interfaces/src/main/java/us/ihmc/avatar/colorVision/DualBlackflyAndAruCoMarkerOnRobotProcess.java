package us.ihmc.avatar.colorVision;

import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.spinnaker.BytedecoBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerHardwareManager;
import us.ihmc.perception.spinnaker.SpinnakerTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;

/** To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 *  the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "00000000");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final ROS1Helper ros1Helper;
   private final SideDependentList<String> serialNumbers = new SideDependentList<>(LEFT_SERIAL_NUMBER, RIGHT_SERIAL_NUMBER);
   private SideDependentList<BytedecoBlackfly> blackflies = new SideDependentList<>();

   private SpinnakerHardwareManager spinnakerHardwareManager;

   public DualBlackflyAndAruCoMarkerOnRobotProcess()
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ros1Helper = new ROS1Helper("blackfly_node");

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            spinnakerHardwareManager = new SpinnakerHardwareManager();
            for (RobotSide side : RobotSide.values)
            {
               BytedecoBlackfly blackfly = spinnakerHardwareManager.buildBlackfly(serialNumbers.get(side));
               blackflies.put(side, blackfly);
               blackfly.initialize();
            }
         }
      }
   }

   private void destroy()
   {
      for (RobotSide side : RobotSide.values)
      {
         blackflies.get(side).destroy();
      }
      spinnakerHardwareManager.destroy();
   }

   public static void main(String[] args)
   {
      SpinnakerTools.printAllConnectedDevicesInformation();
   }
}
