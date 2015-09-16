package us.ihmc.wanderer.operatorInterface;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.wanderer.parameters.WandererRobotModel;

public class WandererOperatorInterface
{
   public static void main(String[] args)
   {
      JSAP jsap = new JSAP();

      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");

      try
      {
         jsap.registerParameter(runningOnRealRobot);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            DRCRobotModel robotModel = new WandererRobotModel(config.getBoolean(runningOnRealRobot.getID()), false);

            try
            {
               Class<?> clazz = Class.forName("us.ihmc.humanoidOperatorInterface.DRCOperatorInterface");
               Method method = clazz.getDeclaredMethod("startUserInterface", us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel.class);
               method.invoke(null, robotModel);
            }
            catch (ClassNotFoundException e)
            {
               e.printStackTrace();
            }
            catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               e.printStackTrace();
            }
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
   }
}
