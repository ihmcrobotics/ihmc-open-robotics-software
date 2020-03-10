package us.ihmc.valkyrie.torquespeedcurve;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieLinkMassMutator implements SDFDescriptionMutator {
	private final Map<String, Double> customLinkMassMap = new HashMap<>();

	public ValkyrieLinkMassMutator(ValkyrieJointMap jointMap, String linkName, double mass) {
		System.out.printf("Changing mass of %s link to %f (note: torso may be further adjusted)\n", linkName, mass);
		customLinkMassMap.put(linkName, mass);
	}

	@Override
	public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder) {

	}

	@Override
	public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder) {
		if (customLinkMassMap.containsKey(linkHolder.getName())) {
			double oldMass = linkHolder.getMass();
			double newMass = customLinkMassMap.get(linkHolder.getName());

			// Update link mass
			linkHolder.setMass(newMass);

			// Scale inertia tensor
			Matrix3D inertia = linkHolder.getInertia();
			inertia.scale(newMass / oldMass);
			linkHolder.setInertia(inertia);
		}
	}

	@Override
	public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor) {
	}

	@Override
	public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor) {
	}

	@Override
	public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor) {
	}

	@Override
	public void mutateModelWithAdditions(GeneralizedSDFRobotModel model) {
	}
}
