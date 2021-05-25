package us.ihmc.valkyrie.jsc;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.directionalControlToolboxModule.DirectionalControlModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class DirectionalControlApplication
{
   public void start(JSAPResult jsapResult)
   {
      String robotTargetString = jsapResult.getString("robotTarget");
      RobotTarget robotTarget = RobotTarget.valueOf(robotTargetString);
      LogTools.info("-------------------------------------------------------------------");
      LogTools.info("  -------- Loading parameters for RobotTarget: " + robotTarget + "  -------");
      LogTools.info("-------------------------------------------------------------------");
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(robotTarget, ValkyrieRosControlController.VERSION);

      new DirectionalControlModule(robotModel, false, PubSubImplementation.FAST_RTPS);
   }

   /**
    * Argument options:
    * <ul>
    * <li>Selecting the robot target: for sim: {@code --robotTarget=SCS}, for hardware:
    * {@code --robotTarget=REAL_ROBOT}.
    * <li>Selecting the working directory (where the profiles are saved):
    * {@code --workingDir=~/home/myWorkingDirectory}. If none provided the default is set to
    * {@code "~/.ihmc/joystick_step_app/"}.
    * </ul>
    * 
    * @param args the array of arguments to use for this run.
    */
   public static void main(String[] args)
   {
      // TODO: Add support for profiles/working directory
      JSAPResult jsapResult = null;
      try
      {
         final SimpleJSAP jsap = new SimpleJSAP("ValkyrieVrSteppingApplication",
                                                "Make the robot walk",
                                                new Parameter[] {new FlaggedOption("robotTarget",
                                                                                   JSAP.STRING_PARSER,
                                                                                   "SCS",
                                                                                   JSAP.NOT_REQUIRED,
                                                                                   'r',
                                                                                   "robotTarget",
                                                                                   "Robot target (REAL_ROBOT, SCS)")});
         jsapResult = jsap.parse(args);
      }
      catch (JSAPException e)
      {
         System.out.println("Invalid option: " + e);
         System.exit(0);
      }
      DirectionalControlApplication app = new DirectionalControlApplication();
      app.start(jsapResult);
   }
}
