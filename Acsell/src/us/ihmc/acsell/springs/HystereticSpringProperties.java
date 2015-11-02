package us.ihmc.acsell.springs;

public interface HystereticSpringProperties extends SpringProperties {

	double getUnloadingSpringConstant();
	double getLoadingSpringConstant();
	double getUnloadingRestLength();
	double getLoadingRestLength();

}
