package us.ihmc.valkyrie.log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;


public class ValkyrieModelBundleGenerator
{
   public static void main(String[] args) throws IOException
   {
      ValkyrieRobotModel model = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, false);
      LogModelProvider logModelProvider = model.getLogModelProvider();
      Properties properties = new Properties();
      properties.setProperty("modelName", logModelProvider.getModelName());
      properties.setProperty("loader", logModelProvider.getLoader().getCanonicalName());
      properties.setProperty("resourceDirectories", StringUtils.join(logModelProvider.getResourceDirectories(), ","));
      
      File dir = new File("../RobotDataCommunication/RobotModels/Valkyrie");
      dir.mkdirs();
      
      File description = new File(dir, "description.properties");
      File modelFile = new File(dir, "model.sdf");
      File resourceFile = new File(dir, "resources.zip");
      
      FileOutputStream modelStream = new FileOutputStream(modelFile);
      modelStream.write(logModelProvider.getModel());
      modelStream.close();
      
      FileOutputStream resourceStream = new FileOutputStream(resourceFile);
      resourceStream.write(logModelProvider.getResourceZip());
      resourceStream.close();
      
      FileWriter writer = new FileWriter(description);
      properties.store(writer, "Created by ValkyrieModelBunderGenerator");
      writer.close();      
   }
}
