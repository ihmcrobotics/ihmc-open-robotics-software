package us.ihmc.communication.ros;



import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Field;
import java.lang.reflect.ParameterizedType;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.sensing.LookAtPacket;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.packets.walking.FootStatePacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.packets.walking.PauseCommand;
import us.ihmc.communication.packets.walking.PelvisPosePacket;

public class ROSMessageGenerator
{
	
	private static String messageFolder= ("../ROSJavaBootstrap/ROSMessagesAndServices/ihmc_msgs/msg/").replace("/", File.separator);
	boolean overwriteSubMessages;

	public ROSMessageGenerator(Boolean overwriteSubMessages)
	{
		this.overwriteSubMessages = overwriteSubMessages;
	}

	public static void main(String... args)
	{
		List<Class> toConvert = new ArrayList<Class>();
		toConvert.add(RobotConfigurationData.class);
		toConvert.add(HandPosePacket.class);
		toConvert.add(ComHeightPacket.class);
		toConvert.add(FootPosePacket.class);
		toConvert.add(FootStatePacket.class);
		//toConvert.add(RobotPoseData.class); //RigidBodyTranspose conversion required
		toConvert.add(FootstepData.class);
		toConvert.add(FootstepDataList.class);
		toConvert.add(FootstepStatus.class);
		toConvert.add(ChestOrientationPacket.class);
		toConvert.add(LookAtPacket.class);
		toConvert.add(HeadOrientationPacket.class);
		toConvert.add(PauseCommand.class);
		toConvert.add(PelvisPosePacket.class);
		
		ROSMessageGenerator messageGenerator = new ROSMessageGenerator(true);
		for (Class clazz : toConvert)
		{
			messageGenerator.createNewRosMessage(clazz, true);
		}
	}

	public String createNewRosMessage(Class clazz, boolean overwrite)
	{
		if (clazz == null)
		{
			return "";
		}
		
		File file = new File(messageFolder);
		if (!file.exists())
		{
			file.mkdirs();
		}
		
		String messageName = clazz.getSimpleName() + "Message";
		File messageFile = new File(messageFolder + "\\" + messageName + ".msg");

		if (overwrite || !messageFile.exists())
		{
			messageFile.delete();
			try
			{
				messageFile.createNewFile();
				System.out.println("Message Created: " + messageFile.getName());
				PrintStream fileStream = new PrintStream(messageFile);

				String outBuffer = "# " + messageName + System.lineSeparator() + System.lineSeparator();

				Field[] fields = clazz.getFields();
				for (Field field : fields)
				{
					outBuffer += printType(field);
					outBuffer += " " + field.getName() + System.lineSeparator();
				}
				fileStream.println(outBuffer);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}
		return messageName;
	}
	
	private String printType(Field field){
		String buffer = "";
		if (List.class.isAssignableFrom(field.getType())){
			Class genericType = ((Class) ((ParameterizedType) field.getGenericType()).getActualTypeArguments()[0]);
			buffer += printType(genericType);
			buffer += "[]";
		}
		else{
			buffer += printType(field.getType());
		}
		
		return buffer;
	}

	private String printType(Class clazz)
	{
		String buffer = "";
		if (clazz == null)
		{
			return buffer;
		}

		if (clazz.isArray())
		{
			buffer += printType(clazz.getComponentType());
			buffer += "[]";
		}      
		else if (clazz.isEnum())
		{
			Object[] enumList = clazz.getEnumConstants();
			buffer += "#Options for enum";
			for (int i = 0; i < enumList.length; i++)
			{
				buffer += "# uint8";
				buffer += " " + enumList[i];
				buffer += " = " + i + System.lineSeparator();
			}
			buffer += "uint8";
		}
		else if (clazz.equals(byte.class)){ buffer += "int8";}
		else if (clazz.equals(short.class)){ buffer += "int16";}
		else if (clazz.equals(int.class)){ buffer += "int32";	}
		else if (clazz.equals(long.class)){	buffer += "int64";	}
		else if (clazz.equals(float.class)){	buffer += "float32";	}
		else if (clazz.equals(double.class)){	buffer += "float64";}
		else if (clazz.equals(boolean.class)){ buffer += "bool";}
		else if (clazz.equals(char.class)){ buffer += "uint8";}
		else if (clazz.equals(String.class)){ buffer += "string";}
		
		else if (clazz.equals(Quat4d.class)){buffer += "geometry_msgs/Quaternion";}
		else if (clazz.equals(Point3d.class) || (clazz.equals(Vector3d.class))){buffer += "geometry_msgs/Vector3";}
		else{
			//buffer += "ihmc_msg/";
			buffer += createNewRosMessage(clazz, overwriteSubMessages);
		}
		return buffer;
	}
}



