package us.ihmc.atlas.log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;


public class AtlasModelBundleGenerator
{
   public static void main(String[] args) throws IOException
   {
      AtlasRobotModel model = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.REAL_ROBOT, false);
      LogModelProvider logModelProvider = model.getLogModelProvider();
      Properties properties = new Properties();
      properties.setProperty("modelName", logModelProvider.getModelName());
      properties.setProperty("loader", logModelProvider.getLoader().getCanonicalName());
      properties.setProperty("resourceDirectories", StringUtils.join(logModelProvider.getResourceDirectories(), ","));
      
      File dir = new File("../RobotDataCommunication/RobotModels/Atlas");
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
      properties.store(writer, "Created by AtlasModelBunderGenerator");
      writer.close();      
   }
}
